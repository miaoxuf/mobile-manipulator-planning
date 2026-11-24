function out = plan_grasp_tasks(cfg)
%PLAN_GRASP_TASKS Energy- and accuracy-aware planning for all grasp tasks.
%   out = PLAN_GRASP_TASKS(cfg) runs the complete pipeline for the
%   configuration struct cfg (see DEFAULT_CONFIG):
%
%     1. working ring of the arm (Monte Carlo workspace or user value)
%     2. station candidates per target: free cells inside the ring with
%        an unobstructed Bresenham line to the target
%     3. exact minimum platform energy (moving, turning, final rotation)
%        to every station / heading from one Dijkstra search over the
%        states (x, y, heading)  - PLATFORM_COST_MAP
%     4. arm optimisation (cfg.arm.solver) of the station / heading
%        combinations in order of increasing platform energy, warm-started
%        from earlier solutions (cfg.arm.warm_start); solutions with
%        err <= cfg.arm.err_tol and no floor violation are feasible
%     5. selection of the combination minimising
%        w_energy * (E_platform + E_arm) + w_error * err
%
%   With cfg.select.prune = true step 4 stops as soon as the platform
%   energy alone (a lower bound of the score, because arm energy and error
%   are non-negative) cannot beat the best score found - an exact branch
%   and bound.  With prune = false every combination is optimised, which
%   is useful for the full comparison figures.
%
%   The returned struct contains the robot, map, ring radius, per-task
%   details (out.tasks) and a summary table (out.summary).

t0 = tic;
rng(cfg.seed);
robot = validate_robot(cfg.robot);
occ   = logical(cfg.env.map);
cs    = cfg.env.cell_size;
targets = cfg.env.targets;
rel_h = cfg.env.target_height - robot.base_height;
check_inputs(occ, cfg.env.start, targets);
say = @(varargin) cfg.verbose && fprintf(varargin{:});

%% 1) working ring -------------------------------------------------------
ws = [];
if isempty(cfg.workspace.radius)
    zb = cfg.workspace.z_band;
    if isempty(zb)
        zb = rel_h + [-100 200];
    end
    ws = compute_workspace(robot, cfg.workspace.num_samples, zb);
    radius = [ws.r_min ws.r_max];
    say('Workspace ring (Monte Carlo, z in [%g, %g] mm): %.0f - %.0f mm\n', zb, radius);
else
    radius = cfg.workspace.radius;
    say('Workspace ring (user defined): %.0f - %.0f mm\n', radius);
end
radius = radius + [1 -1] * cfg.workspace.margin;

orients = cfg.platform.orientations(:).';
nO = numel(orients);
solver = cfg.arm.solver;
if ischar(solver) || isstring(solver)
    solver = str2func(solver);
end
takes_seeds = solver_accepts_seeds(solver);
n = size(robot.dh, 1);
w = cfg.select;
assert(w.w_energy >= 0 && w.w_error >= 0, 'cfg.select weights must be non-negative.');
prune = getfield_or(cfg.select, 'prune', false);
warm  = getfield_or(cfg.arm, 'warm_start', false) && takes_seeds;
yaw_sym = warm && has_base_yaw_symmetry(robot);

% one Dijkstra search gives the exact platform energy to every state
cmap = platform_cost_map(occ, cfg.env.start, cfg.env.start_heading, cfg.platform);

%% 2-5) tasks ------------------------------------------------------------
tasks = repmat(empty_task(), size(targets, 1), 1);
for ti = 1:size(targets, 1)
    tgt  = targets(ti, :);
    cand = find_station_candidates(occ, tgt, radius, cs);
    st   = cand.valid;
    nS   = size(st, 1);
    say('Task %d  target (%d,%d): %d cells in ring, %d unobstructed stations\n', ...
        ti, tgt, size(cand.ring, 1), nS);

    [S_idx, O_idx] = ndgrid(1:nS, 1:nO);
    S_idx = S_idx(:);  O_idx = O_idx(:);
    nJ = numel(S_idx);
    stations = st(S_idx, :);
    if nJ == 0
        stations = zeros(0, 2);
    end
    heads = reshape(orients(O_idx), [], 1);

    % exact platform energy = lower bound of every combination's score
    Eplat = platform_energy_bound(cmap, stations, heads);
    [~, order] = sort(Eplat);

    err  = nan(nJ, 1);  Earm = nan(nJ, 1);  viol = nan(nJ, 1);
    Q    = nan(nJ, n);  sols = cell(nJ, 1);
    evaluated = false(nJ, 1);
    lib = struct('target', zeros(0, 3), 'q', zeros(0, n), 'ok', false(0, 1));
    best_score = inf;

    % arm optimisation, cheapest platform energy first (branch and bound)
    for j = order(:).'
        if isinf(Eplat(j))
            continue                                    % station unreachable
        end
        if prune && w.w_energy * Eplat(j) >= best_score
            break                                       % no better score possible
        end
        local = local_target(tgt, stations(j, :), heads(j), cs, rel_h);
        if warm
            seeds = warm_start_seeds(lib, local, robot, cfg.arm.warm_k, yaw_sym);
            sol = solver(robot, local, cfg.arm, seeds);
        else
            sol = solver(robot, local, cfg.arm);
        end
        sols{j} = sol;
        evaluated(j) = true;
        err(j)  = sol.err;
        Earm(j) = sol.energy;
        viol(j) = getfield_or(sol, 'floor_violation', 0);
        Q(j, :) = sol.q(:).';
        ok = err(j) <= cfg.arm.err_tol && viol(j) <= 1e-6;
        if ok
            best_score = min(best_score, w.w_energy * (Eplat(j) + Earm(j)) + w.w_error * err(j));
        end
        lib.target(end + 1, :) = local;
        lib.q(end + 1, :) = sol.q(:).';
        lib.ok(end + 1, 1) = ok;
    end
    feasible = evaluated & err <= cfg.arm.err_tol & viol <= 1e-6;

    % platform paths and energy breakdown for feasible combinations
    Emove = nan(nJ, 1); Eturn = nan(nJ, 1); Erot = nan(nJ, 1);
    paths = cell(nJ, 1);
    for j = find(feasible).'
        pr = extract_platform_path(cmap, stations(j, :), heads(j));
        paths{j} = pr;
        Emove(j) = pr.E_move;  Eturn(j) = pr.E_turn;  Erot(j) = pr.E_rot;
    end
    Etotal = Eplat + Earm;
    score  = w.w_energy * Etotal + w.w_error * err;
    score(~feasible) = inf;

    evals = table(stations(:, 1), stations(:, 2), heads, evaluated, err, Earm, feasible, ...
                  Emove, Eturn, Erot, Eplat, Etotal, score, ...
        'VariableNames', {'station_x', 'station_y', 'orientation', 'evaluated', 'error', ...
                          'E_arm', 'feasible', 'E_move', 'E_turn', 'E_rot', ...
                          'E_platform', 'E_total', 'score'});

    T = empty_task();
    T.id = ti;
    T.target = tgt;
    T.candidates = cand;
    T.evals = evals;
    T.Q = Q;
    T.solutions = sols;
    T.paths = paths;
    T.n_valid = nS;
    T.n_evaluated = nnz(evaluated);
    T.n_feasible = size(unique(stations(feasible, :), 'rows'), 1);

    if any(feasible)
        [~, b] = min(score);
        best.row         = b;
        best.station     = stations(b, :);
        best.orientation = heads(b);
        best.q           = Q(b, :);
        best.target_local = local_target(tgt, best.station, best.orientation, cs, rel_h);
        best.error       = err(b);
        best.E_arm       = Earm(b);
        best.E_move      = Emove(b);
        best.E_turn      = Eturn(b);
        best.E_rot       = Erot(b);
        best.E_platform  = Eplat(b);
        best.E_total     = Etotal(b);
        best.path        = paths{b};
        best.traj        = arm_trajectory(robot, Q(b, :), cfg.arm);
        best.history     = getfield_or(sols{b}, 'history', struct());
        T.best  = best;
        T.found = true;
        say('        -> %d of %d combinations optimised, %d feasible stations; best (%d,%d) @ %g deg, error %.2f mm, energy %.1f J\n', ...
            T.n_evaluated, nJ, T.n_feasible, best.station, best.orientation, best.error, best.E_total);
    else
        say('        -> no station satisfies the error tolerance of %g mm\n', cfg.arm.err_tol);
    end
    tasks(ti) = T;
end

out.cfg       = cfg;
out.robot     = robot;
out.map       = occ;
out.workspace = ws;
out.radius    = radius;
out.tasks     = tasks;
out.summary   = summary_table(tasks);
out.runtime   = toc(t0);
say('Finished in %.1f s.\n', out.runtime);
end

% ------------------------------------------------------------------------
function p = local_target(target, station, heading, cs, rel_h)
% Target position in the arm base frame of a platform at `station`
% facing `heading` (deg, measured from +x towards +y).
d = (target(:) - station(:)) * cs;
c = cosd(heading);  s = sind(heading);
p = [c * d(1) + s * d(2), -s * d(1) + c * d(2), rel_h];
end

function v = getfield_or(s, name, default)
if isfield(s, name), v = s.(name); else, v = default; end
end

function tf = solver_accepts_seeds(solver)
% True if the solver takes a fourth input (warm-start seeds).
try
    na = nargin(solver);
catch
    na = 3;
end
tf = na < 0 || na >= 4;
end

function T = empty_task()
T = struct('id', 0, 'target', [0 0], 'candidates', [], 'evals', table(), ...
           'Q', [], 'solutions', {{}}, 'paths', {{}}, 'n_valid', 0, ...
           'n_evaluated', 0, 'n_feasible', 0, 'best', [], 'found', false);
end

function tbl = summary_table(tasks)
n = numel(tasks);
v = nan(n, 12);
for i = 1:n
    v(i, 1:2) = tasks(i).target;
    v(i, 3:5) = [tasks(i).n_valid, tasks(i).n_evaluated, tasks(i).n_feasible];
    if tasks(i).found
        b = tasks(i).best;
        v(i, 6:12) = [b.station, b.orientation, b.error, b.E_arm, b.E_platform, b.E_total];
    end
end
tbl = array2table(v, 'VariableNames', {'target_x', 'target_y', 'n_valid', ...
    'n_evaluated', 'n_feasible', 'station_x', 'station_y', 'orientation', ...
    'error_mm', 'E_arm_J', 'E_platform_J', 'E_total_J'});
end

function check_inputs(occ, start, targets)
sz = size(occ);
inside = @(p) all(p >= 1) && p(1) <= sz(1) && p(2) <= sz(2);
assert(inside(start), 'cfg.env.start lies outside the map.');
assert(~occ(start(1), start(2)), 'cfg.env.start lies on an obstacle.');
for i = 1:size(targets, 1)
    assert(inside(targets(i, :)), 'Target %d lies outside the map.', i);
end
end
