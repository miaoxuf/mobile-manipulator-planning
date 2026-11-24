function [q, J, hist] = nullspace_refine(robot, q, target, arm, opt)
%NULLSPACE_REFINE Local refinement along the inverse-kinematics manifold.
%   [q, J, hist] = NULLSPACE_REFINE(robot, q0, target, arm, opt) improves a
%   configuration that already reaches (or nearly reaches) the target.
%   For a redundant positioning task the solutions form a curved manifold
%   on which coordinate-wise or population-based search converges slowly.
%   Each iteration therefore
%     1. pulls the end-effector onto the target with a damped
%        least-squares step  dq_c = J^T (J J^T + lambda^2 I)^-1 e,
%     2. descends the arm energy inside the null space of the position
%        Jacobian,           dq_n = -(I - J^+ J) grad E,
%     3. accepts the step by backtracking on the full ARM_COST.
%   Jacobian and gradient are evaluated by batched finite differences.
%
%   opt fields: max_iter (40), step (5 deg, max null-space step),
%   lambda (damping, 1e-2 * mean |J|), tol (1e-9 relative improvement).

if nargin < 5, opt = struct(); end
max_iter = field_or(opt, 'max_iter', 40);
max_step = field_or(opt, 'step', 5);
tol      = field_or(opt, 'tol', 1e-9);

lo = robot.qlim(:, 1).';
hi = robot.qlim(:, 2).';
n  = numel(q);
h  = 1e-4;                                  % finite-difference step (deg)
cost = @(Q) arm_cost(robot, Q, target, arm);
J = cost(q);
hist = J;
alpha = max_step;

for it = 1:max_iter
    Qh = [q; q + h * eye(n)];
    P  = fk_batch(robot, Qh);
    Jac = (P(2:end, :) - P(1, :)).' / h;                     % 3-by-n (mm/deg)
    E  = arm_energy(robot, Qh, arm.T_nominal);
    g  = (E(2:end) - E(1)).' / h;                            % 1-by-n (J/deg)
    e  = target(:) - P(1, :).';

    lam = field_or(opt, 'lambda', 1e-2 * mean(abs(Jac(:))) + eps);
    A    = Jac * Jac.' + lam^2 * eye(3);
    Jpin = Jac.' / A;                                        % n-by-3
    dq_c = (Jpin * e).';
    dq_n = -((eye(n) - Jpin * Jac) * g.').';
    sn = max(abs(dq_n));
    if sn > 0
        dq_n = dq_n / sn;                                    % unit max step
    end

    improved = false;
    a = alpha;
    for ls = 1:12
        qn = min(max(q + dq_c + a * dq_n, lo), hi);
        Jn = cost(qn);
        if Jn < J
            improved = true;
            break
        end
        a = a / 2;
    end
    if ~improved
        break
    end
    rel = (J - Jn) / max(1, abs(J));
    q = qn;
    J = Jn;
    hist(end + 1, 1) = J; %#ok<AGROW>
    alpha = min(max_step, 2 * a);                            % adapt step size
    if rel < tol
        break
    end
end
end

function v = field_or(s, name, default)
if isfield(s, name), v = s.(name); else, v = default; end
end
