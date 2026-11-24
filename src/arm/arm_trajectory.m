function traj = arm_trajectory(robot, q_goal, arm)
%ARM_TRAJECTORY Quintic joint trajectory from the home pose to q_goal.
%   traj = ARM_TRAJECTORY(robot, q_goal, arm) samples the rest-to-rest
%   fifth-order polynomial (zero velocity and acceleration at both ends)
%   with step arm.dt and integrates the joint power numerically.
%
%   Output struct (angles in deg, rates in deg/s and deg/s^2):
%     t, q, qd, qdd  - time samples and joint profiles
%     power          - |I .* qdd .* qd| per joint (W)
%     energy_joint   - 1-by-n energy per joint (J)
%     energy         - total energy (J)
%     T              - motion duration (s)
%     ee             - end-effector path in the base frame (mm)

q0 = robot.q_home;
dq = deg2rad(q_goal(:).' - q0);
T  = motion_time(robot, dq, arm.T_nominal);

t = (0:arm.dt:T).';
if t(end) < T
    t(end + 1) = T;
end
s   = t / T;
pos = 10 * s.^3 - 15 * s.^4 + 6 * s.^5;
vel = (30 * s.^2 - 60 * s.^3 + 30 * s.^4) / T;
acc = (60 * s - 180 * s.^2 + 120 * s.^3) / T^2;

q   = deg2rad(q0) + pos .* dq;      % rad
qd  = vel .* dq;                    % rad/s
qdd = acc .* dq;                    % rad/s^2
power = abs(robot.inertia .* qdd .* qd);
Ej = trapz(t, power, 1);

traj.t            = t;
traj.q            = rad2deg(q);
traj.qd           = rad2deg(qd);
traj.qdd          = rad2deg(qdd);
traj.power        = power;
traj.energy_joint = Ej;
traj.energy       = sum(Ej);
traj.T            = T;
traj.ee           = fk_batch(robot, traj.q);
end
