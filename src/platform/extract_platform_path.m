function res = extract_platform_path(cm, goal, goal_heading)
%EXTRACT_PLATFORM_PATH Optimal platform path from a PLATFORM_COST_MAP.
%   res = EXTRACT_PLATFORM_PATH(cm, goal, goal_heading) returns the
%   minimum-energy path to cell `goal` ending with heading `goal_heading`
%   (deg, or NaN = any), including the final in-place rotation.  The
%   result struct is identical to the one of ENERGY_ASTAR.
%
%   E = EXTRACT_PLATFORM_PATH(cm, goal, goal_heading) is cheap; use
%   PLATFORM_ENERGY_BOUND when only the energy is needed.

R = cm.size(1);  C = cm.size(2);
gk = reshape(cm.g(goal(1), goal(2), :), 1, 4) + ...
     cm.p.k_rot * heading_delta(90 * (0:3), goal_heading);
[best, k] = min(gk);
if isinf(best)
    res = platform_path_result([], R, C, goal_heading, cm.p);
    return
end
s = goal(1) + (goal(2) - 1) * R + (k - 1) * R * C;
chain = zeros(0, 1);
while s ~= 0
    chain(end + 1, 1) = s; %#ok<AGROW>
    s = cm.parent(s);
end
res = platform_path_result(flipud(chain), R, C, goal_heading, cm.p);
end
