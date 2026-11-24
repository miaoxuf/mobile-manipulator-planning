function T = motion_time(robot, dq, T_nominal)
%MOTION_TIME Duration of a rest-to-rest quintic joint motion.
%   T = MOTION_TIME(robot, dq, T_nominal) returns, for every row of the
%   N-by-n joint displacement matrix dq (rad), the shortest duration that
%   is not below T_nominal and respects the average joint speed
%   robot.omega_avg and the peak joint acceleration robot.alpha_max.
%
%   For a quintic profile the peak acceleration is 10/sqrt(3)*|dq|/T^2.

dq = abs(dq);
t_speed = max(dq ./ robot.omega_avg, [], 2);
t_acc   = max(sqrt((10 / sqrt(3)) * dq ./ robot.alpha_max), [], 2);
T = max([repmat(T_nominal, size(dq, 1), 1), t_speed, t_acc], [], 2);
end
