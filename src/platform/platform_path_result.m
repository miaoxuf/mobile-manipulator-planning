function res = platform_path_result(states, R, C, goal_heading, p)
%PLATFORM_PATH_RESULT Path struct and energy breakdown from a state chain.
%   res = PLATFORM_PATH_RESULT(states, R, C, goal_heading, p) converts the
%   ordered state indices (x, y, heading) of an R-by-C grid into the result
%   struct returned by ENERGY_ASTAR and EXTRACT_PLATFORM_PATH.  An empty
%   chain yields found = false.

if isempty(states)
    res = struct('found', false, 'path', zeros(0, 2), 'headings', zeros(0, 1), ...
                 'final_heading', goal_heading, 'E_move', inf, 'E_turn', inf, ...
                 'E_rot', inf, 'E_total', inf, 'length', inf);
    return
end
states = states(:);
k = floor((states - 1) / (R * C));
r = states - 1 - k * R * C;
headings = 90 * k;

res.found    = true;
res.path     = [mod(r, R) + 1, floor(r / R) + 1];
res.headings = headings;
if isnan(goal_heading)
    res.final_heading = headings(end);
else
    res.final_heading = goal_heading;
end
res.length  = numel(states) - 1;
res.E_move  = p.k_move * res.length;
res.E_turn  = p.k_turn * sum(heading_delta(headings(1:end-1), headings(2:end)));
res.E_rot   = p.k_rot  * heading_delta(headings(end), goal_heading);
res.E_total = res.E_move + res.E_turn + res.E_rot;
end
