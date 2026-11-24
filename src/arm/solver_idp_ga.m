function sol = solver_idp_ga(robot, target, arm, seeds)
%SOLVER_IDP_GA Default grasp solver: multi-start IDP -> GA -> local refinement.
%   Stages: (1) multi-start iterative dynamic programming, (2) genetic
%   algorithm seeded with the IDP results, (3) short local IDP and
%   (4) null-space gradient refinement on the IK manifold (NULLSPACE_REFINE).
%   sol = SOLVER_IDP_GA(robot, target, arm) searches the joint
%   configuration that reaches `target` (1-by-3, mm, arm base frame) with
%   minimum ARM_COST, starting from robot.q_home.
%   sol = SOLVER_IDP_GA(robot, target, arm, seeds) additionally starts
%   from the warm-start configurations in `seeds` (k-by-n, deg), e.g.
%   solutions of nearby targets supplied by PLAN_GRASP_TASKS.  With seeds
%   only arm.idp.starts_warm random starts are added instead of
%   arm.idp.starts - 1.
%
%   This function defines the solver interface used by PLAN_GRASP_TASKS.
%   Any function with the signature f(robot, target, arm) or
%   f(robot, target, arm, seeds) can be plugged in through cfg.arm.solver,
%   e.g. a wrapper around fmincon, ga or particleswarm.  It must return a
%   struct with at least the fields
%     q       1-by-n joint configuration (deg)
%     err     end-effector position error (mm)
%     energy  arm energy (J)
%     cost    objective value
%   and may add the fields `floor_violation` (mm, 0 if the pose keeps the
%   floor clearance) and `history` (struct of convergence traces).

if nargin < 4
    seeds = zeros(0, size(robot.dh, 1));
end
lo = robot.qlim(:, 1).';
hi = robot.qlim(:, 2).';
cost_fn = @(Q) arm_cost(robot, Q, target, arm);

% 1) multi-start iterative dynamic programming (all starts in one batch)
if isempty(seeds)
    nRand = max(0, arm.idp.starts - 1);
else
    nRand = arm.idp.starts_warm;
end
Q0 = [robot.q_home; min(max(seeds, lo), hi); lo + (hi - lo) .* rand(nRand, numel(lo))];
[starts, ~, h_idp] = idp_search(cost_fn, Q0, lo, hi, arm.idp);

% 2) genetic algorithm seeded with all IDP results (best first)
[q, ~, h_ga] = ga_search(cost_fn, starts, lo, hi, arm.ga);

% 3) local polish in a small region around the GA optimum
h_pol = zeros(0, 1);
if arm.polish
    [q, ~, h_pol] = idp_search(cost_fn, q, lo, hi, arm.idp, 0.05 * (hi - lo));
end

% 4) null-space gradient refinement along the inverse-kinematics manifold
h_ns = zeros(0, 1);
if isfield(arm, 'nullspace') && arm.nullspace.enable
    [q, ~, h_ns] = nullspace_refine(robot, q, target, arm, arm.nullspace);
end

[J, err, E, v] = arm_cost(robot, q, target, arm);
sol.q      = q;
sol.err    = err;
sol.energy = E;
sol.cost   = J;
sol.floor_violation = v;
sol.history = struct('idp', h_idp, 'ga', h_ga, 'polish', h_pol, 'nullspace', h_ns);
end
