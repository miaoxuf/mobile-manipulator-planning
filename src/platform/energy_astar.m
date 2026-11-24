function res = energy_astar(occ, start, start_heading, goal, goal_heading, p)
%ENERGY_ASTAR Minimum-energy path of the mobile platform on a 4-connected grid.
%   res = ENERGY_ASTAR(occ, start, start_heading, goal, goal_heading, p)
%   plans from cell `start` with heading `start_heading` (deg, multiple of
%   90, or NaN = free) to cell `goal`, where the platform must finally
%   face `goal_heading` (deg, or NaN = any).
%
%   The search runs on states (x, y, heading) so that the heading-dependent
%   turning cost is handled exactly.  Edge and terminal costs:
%     move one cell straight ahead      p.k_move
%     change travel direction           p.k_turn * |delta heading| (deg)
%     final in-place rotation at goal   p.k_rot  * |delta heading| (deg)
%   The heuristic p.k_move * Manhattan distance is consistent, hence the
%   returned path is energy optimal.  The open list is a binary heap.
%
%   For many goals from the same start use PLATFORM_COST_MAP and
%   EXTRACT_PLATFORM_PATH, which share one Dijkstra search.
%
%   Output struct:
%     found      - true if a path exists
%     path       - L-by-2 cells from start to goal
%     headings   - L-by-1 travel heading at every cell (deg)
%     final_heading, E_move, E_turn, E_rot, E_total (J), length (cells)

DIRS = [1 0; 0 1; -1 0; 0 -1];      % heading index 0..3 -> 0, 90, 180, 270 deg
[R, C] = size(occ);
RC   = R * C;
S    = RC * 4;
DONE = S + 1;                        % virtual terminal state

if occ(start(1), start(2)) || occ(goal(1), goal(2))
    res = platform_path_result([], R, C, goal_heading, p);
    return
end

g      = inf(S + 1, 1);
closed = false(S + 1, 1);
parent = zeros(S + 1, 1);
heap   = MinHeap(1024);
turnCost = p.k_turn * [0 90 180 90];            % by mod(d - k, 4)
rotCost  = p.k_rot * heading_delta(90 * (0:3), goal_heading);

if isnan(start_heading)
    k0 = 0:3;
else
    k0 = mod(round(start_heading / 90), 4);
end
h0 = p.k_move * (abs(start(1) - goal(1)) + abs(start(2) - goal(2)));
for k = k0
    s = start(1) + (start(2) - 1) * R + k * RC;
    g(s) = 0;
    heap.push(h0, s);
end

found = false;
while ~heap.is_empty()
    [~, s] = heap.pop();
    if closed(s)
        continue
    end
    if s == DONE
        found = true;
        break
    end
    closed(s) = true;
    k = floor((s - 1) / RC);
    r = s - 1 - k * RC;
    x = mod(r, R) + 1;
    y = floor(r / R) + 1;

    if x == goal(1) && y == goal(2)
        gd = g(s) + rotCost(k + 1);
        if gd < g(DONE)
            g(DONE) = gd;
            parent(DONE) = s;
            heap.push(gd, DONE);
        end
    end

    for d = 0:3
        nx = x + DIRS(d + 1, 1);
        ny = y + DIRS(d + 1, 2);
        if nx < 1 || nx > R || ny < 1 || ny > C || occ(nx, ny)
            continue
        end
        ns = nx + (ny - 1) * R + d * RC;
        if closed(ns)
            continue
        end
        gn = g(s) + p.k_move + turnCost(mod(d - k, 4) + 1);
        if gn < g(ns)
            g(ns) = gn;
            parent(ns) = s;
            heap.push(gn + p.k_move * (abs(nx - goal(1)) + abs(ny - goal(2))), ns);
        end
    end
end

chain = [];
if found
    s = parent(DONE);
    while s ~= 0
        chain(end + 1, 1) = s; %#ok<AGROW>
        s = parent(s);
    end
    chain = flipud(chain);
end
res = platform_path_result(chain, R, C, goal_heading, p);
end
