function tf = has_base_yaw_symmetry(robot)
%HAS_BASE_YAW_SYMMETRY True if joint 1 rotates the whole arm about base z.
%   For such arms (every standard D-H arm, and modified D-H arms with
%   a_0 = alpha_0 = 0) a target rotated by an angle phi about the vertical
%   base axis is reached by the same configuration with q1 + phi.  The
%   planner uses this to warm-start the arm optimiser from earlier
%   solutions.  The property is checked numerically.

rng_state = rng;
cleanup = onCleanup(@() rng(rng_state));
rng(0);
n  = size(robot.dh, 1);
lo = robot.qlim(:, 1).';
hi = robot.qlim(:, 2).';
Q  = lo + (hi - lo) .* rand(8, n);
phi = 360 * rand(8, 1) - 180;
P0 = fk_batch(robot, Q);
Q1 = Q;
Q1(:, 1) = Q1(:, 1) + phi;
P1 = fk_batch(robot, Q1);
c = cosd(phi);  s = sind(phi);
Prot = [c .* P0(:, 1) - s .* P0(:, 2), s .* P0(:, 1) + c .* P0(:, 2), P0(:, 3)];
tf = max(abs(P1 - Prot), [], 'all') < 1e-6 * max(1, max(abs(P0), [], 'all'));
end
