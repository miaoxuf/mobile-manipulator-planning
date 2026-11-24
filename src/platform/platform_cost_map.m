function cm = platform_cost_map(occ, start, start_heading, p)
%PLATFORM_COST_MAP Minimum platform energy from the start to every state.
%   cm = PLATFORM_COST_MAP(occ, start, start_heading, p) runs one Dijkstra
%   search over the states (x, y, heading) of the 4-connected grid with the
%   same moving / turning costs as ENERGY_ASTAR.  Afterwards the optimal
%   path to any station and final heading is obtained in O(path length)
%   with EXTRACT_PLATFORM_PATH, so evaluating hundreds of candidate
%   stations costs a single search instead of one A* run each.
%
%   cm.g(x, y, k+1) is the minimum energy to reach cell (x, y) travelling
%   with heading 90*k deg (Inf if unreachable).

DIRS = [1 0; 0 1; -1 0; 0 -1];
[R, C] = size(occ);
RC = R * C;
S  = RC * 4;

g      = inf(S, 1);
closed = false(S, 1);
parent = zeros(S, 1);
heap   = MinHeap(1024);
turnCost = p.k_turn * [0 90 180 90];

if ~occ(start(1), start(2))
    if isnan(start_heading)
        k0 = 0:3;
    else
        k0 = mod(round(start_heading / 90), 4);
    end
    for k = k0
        s = start(1) + (start(2) - 1) * R + k * RC;
        g(s) = 0;
        heap.push(0, s);
    end
end

while ~heap.is_empty()
    [gs, s] = heap.pop();
    if closed(s) || gs > g(s)
        continue
    end
    closed(s) = true;
    k = floor((s - 1) / RC);
    r = s - 1 - k * RC;
    x = mod(r, R) + 1;
    y = floor(r / R) + 1;
    for d = 0:3
        nx = x + DIRS(d + 1, 1);
        ny = y + DIRS(d + 1, 2);
        if nx < 1 || nx > R || ny < 1 || ny > C || occ(nx, ny)
            continue
        end
        ns = nx + (ny - 1) * R + d * RC;
        gn = gs + p.k_move + turnCost(mod(d - k, 4) + 1);
        if gn < g(ns)
            g(ns) = gn;
            parent(ns) = s;
            heap.push(gn, ns);
        end
    end
end

cm.g      = reshape(g, R, C, 4);
cm.parent = parent;
cm.size   = [R C];
cm.p      = p;
cm.start  = start;
cm.start_heading = start_heading;
end
