function [E, Ej, T] = arm_energy(robot, Q, T_nominal)
%ARM_ENERGY Energy of moving the arm from its home pose to Q.
%   [E, Ej, T] = ARM_ENERGY(robot, Q, T_nominal) evaluates, for each of the
%   N configurations in Q (N-by-n, deg), the energy of a rest-to-rest
%   quintic motion starting at robot.q_home:
%
%       E = sum_i  integral_0^T | I_i * qdd_i(t) * qd_i(t) | dt
%
%   For the quintic profile qd_i is single-signed, so the integral equals
%   I_i * qd_peak_i^2 with qd_peak_i = 15/8 * dq_i / T.  This closed form is
%   identical to the numerical integration in ARM_TRAJECTORY and is used
%   inside the optimiser for speed.
%
%   E  - N-by-1 total energy (J)
%   Ej - N-by-n energy per joint (J)
%   T  - N-by-1 motion durations (s)

dq = deg2rad(Q - robot.q_home);
T  = motion_time(robot, dq, T_nominal);
qd_peak = (15 / 8) * dq ./ T;
Ej = robot.inertia .* qd_peak .^ 2;
E  = sum(Ej, 2);
end
