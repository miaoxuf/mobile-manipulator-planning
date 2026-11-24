function [blocked, cells] = is_line_blocked(occ, station, target)
%IS_LINE_BLOCKED True if an obstacle lies between a station and a target.
%   blocked = IS_LINE_BLOCKED(occ, station, target) traces the Bresenham
%   line from the station cell to the target cell.  The target cell
%   itself is excluded, so goods may be placed on a console (an obstacle
%   cell) and still be grasped from a free neighbouring station.

cells = bresenham_cells(station, target);
inner = cells(1:end-1, :);
inside = inner(:, 1) >= 1 & inner(:, 1) <= size(occ, 1) & ...
         inner(:, 2) >= 1 & inner(:, 2) <= size(occ, 2);
inner = inner(inside, :);
blocked = any(occ(sub2ind(size(occ), inner(:, 1), inner(:, 2))));
end
