function occ = warehouse_map()
%WAREHOUSE_MAP 20 x 20 example warehouse (one cell = 300 mm x 300 mm).
%   occ = WAREHOUSE_MAP() returns a logical occupancy grid with shelves
%   on both sides and a central block of operator consoles.  occ(x, y)
%   is true for an obstacle.  Draw your own layout with MAP_FROM_ASCII.

rows = {
    '.......#####........'   % x = 1
    '.##....#####....###.'
    '.##.............###.'
    '.##.............###.'
    '.##....#####....###.'   % x = 5
    '.##...#######...###.'
    '.......#####........'
    '.##....#####....###.'
    '.##...#######...###.'
    '.##....#####....###.'   % x = 10
    '.##....#####....###.'
    '......#######.......'
    '.##....#####....###.'
    '.##....#####....###.'
    '.##...#######...###.'   % x = 15
    '.##....#####....###.'
    '.......#####........'
    '.##.............###.'
    '.##.............###.'
    '......#######.......'   % x = 20
    };
occ = map_from_ascii(rows);
end
