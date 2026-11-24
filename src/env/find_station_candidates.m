function cand = find_station_candidates(occ, target, radius, cell_size)
%FIND_STATION_CANDIDATES Feasible platform stations for one grasp target.
%   cand = FIND_STATION_CANDIDATES(occ, target, radius, cell_size) scans
%   every free cell of the occupancy grid and keeps those whose distance to
%   the target lies inside the working ring radius = [r_min r_max] (mm).
%
%   Output struct:
%     cand.ring    - K1-by-2 free cells inside the ring
%     cand.blocked - logical K1-by-1, line of sight obstructed
%     cand.valid   - K2-by-2 cells inside the ring and not obstructed
%     cand.dist    - K2-by-1 station-target distance (mm)

[X, Y] = ndgrid(1:size(occ, 1), 1:size(occ, 2));
d = hypot(X - target(1), Y - target(2)) * cell_size;
inRing = ~occ & d >= radius(1) & d <= radius(2);

ring = [X(inRing), Y(inRing)];
dist = d(inRing);
blocked = false(size(ring, 1), 1);
for k = 1:size(ring, 1)
    blocked(k) = is_line_blocked(occ, ring(k, :), target);
end

cand.ring    = ring;
cand.blocked = blocked;
cand.valid   = ring(~blocked, :);
cand.dist    = dist(~blocked);
end
