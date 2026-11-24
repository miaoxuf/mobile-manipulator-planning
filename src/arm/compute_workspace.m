function ws = compute_workspace(robot, num_samples, z_band)
%COMPUTE_WORKSPACE Monte Carlo estimate of the reachable workspace.
%   ws = COMPUTE_WORKSPACE(robot, num_samples, z_band) samples joint
%   configurations uniformly inside the joint limits, evaluates the
%   forward kinematics and derives the horizontal working ring of the
%   points whose height lies in z_band = [z_lo z_hi] (mm, base frame).
%
%   Output struct:
%     ws.points  - num_samples-by-3 end-effector positions (mm)
%     ws.in_band - logical mask of points inside z_band
%     ws.z_band  - the height band used
%     ws.r_min   - minimum horizontal radius inside the band (mm)
%     ws.r_max   - maximum horizontal radius inside the band (mm)

lo = robot.qlim(:, 1).';
hi = robot.qlim(:, 2).';
Q  = lo + (hi - lo) .* rand(num_samples, numel(lo));
P  = fk_batch(robot, Q);

in = P(:, 3) >= z_band(1) & P(:, 3) <= z_band(2);
if ~any(in)
    error('compute_workspace:empty', ...
          ['No sampled end-effector position lies in z = [%g, %g] mm. ' ...
           'Check the D-H table, the joint limits or cfg.workspace.z_band.'], z_band);
end
r = hypot(P(in, 1), P(in, 2));

ws.points  = P;
ws.in_band = in;
ws.z_band  = z_band;
ws.r_min   = min(r);
ws.r_max   = max(r);
end
