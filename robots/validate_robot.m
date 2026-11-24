function robot = validate_robot(robot)
%VALIDATE_ROBOT Check a robot struct and fill in missing optional fields.

assert(isfield(robot, 'dh') && isnumeric(robot.dh) && ...
       any(size(robot.dh, 2) == [3 4]), ...
       'robot.dh must be an n-by-3 [a alpha d] or n-by-4 [a alpha d offset] matrix.');
if size(robot.dh, 2) == 3
    robot.dh = [robot.dh, zeros(size(robot.dh, 1), 1)];
end
n = size(robot.dh, 1);

defaults = struct( ...
    'name',        'custom arm', ...
    'convention',  'standard', ...
    'qlim',        repmat([-180 180], n, 1), ...
    'q_home',      zeros(1, n), ...
    'inertia',     ones(1, n), ...
    'omega_avg',   ones(1, n), ...
    'alpha_max',   inf(1, n), ...
    'base_height', 0);
fn = fieldnames(defaults);
for k = 1:numel(fn)
    if ~isfield(robot, fn{k}) || isempty(robot.(fn{k}))
        robot.(fn{k}) = defaults.(fn{k});
    end
end

robot.convention = validatestring(robot.convention, {'standard', 'modified'});
robot.qlim       = reshape(robot.qlim, n, 2);
robot.q_home     = reshape(robot.q_home, 1, n);
robot.inertia    = expand(robot.inertia, n, 'inertia');
robot.omega_avg  = expand(robot.omega_avg, n, 'omega_avg');
robot.alpha_max  = expand(robot.alpha_max, n, 'alpha_max');

assert(all(robot.qlim(:, 1) < robot.qlim(:, 2)), 'robot.qlim: lower limit must be below upper limit.');
assert(all(robot.omega_avg > 0) && all(robot.alpha_max > 0), ...
       'robot.omega_avg and robot.alpha_max must be positive.');
outside = robot.q_home < robot.qlim(:, 1).' | robot.q_home > robot.qlim(:, 2).';
if any(outside)
    error('validate_robot:home', 'robot.q_home violates the joint limits of joint(s) %s.', ...
          mat2str(find(outside)));
end
end

function v = expand(v, n, name)
v = reshape(v, 1, []);
if isscalar(v)
    v = repmat(v, 1, n);
end
assert(numel(v) == n, 'robot.%s must have one entry per joint (%d).', name, n);
end
