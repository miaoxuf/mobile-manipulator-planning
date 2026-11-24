function [J, err, E, floor_violation] = arm_cost(robot, Q, target, arm)
%ARM_COST Weighted grasp objective for a batch of joint configurations.
%   [J, err, E] = ARM_COST(robot, Q, target, arm) evaluates
%
%       J = w_energy * E + w_error * err
%           + penalty * ( max(0, err - err_tol)^2 + v^2 )
%
%   where err is the Euclidean distance (mm) between the end-effector and
%   the target (1-by-3, base frame), E the motion energy (J) of
%   ARM_ENERGY and v (mm) the largest amount by which a joint frame of the
%   goal pose lies below arm.floor_clearance above the floor (set
%   floor_clearance to -Inf to disable).  Q is N-by-n (deg); all outputs
%   are N-by-1.

[P, ~, Zj] = fk_batch(robot, Q);
err = sqrt(sum((P - target(:).').^2, 2));
E   = arm_energy(robot, Q, arm.T_nominal);

floor_violation = zeros(size(err));
if isfield(arm, 'floor_clearance') && isfinite(arm.floor_clearance)
    floor_violation = max(0, max(arm.floor_clearance - (Zj + robot.base_height), [], 2));
end

J = arm.w_energy * E + arm.w_error * err + ...
    arm.penalty * (max(0, err - arm.err_tol).^2 + floor_violation.^2);
end
