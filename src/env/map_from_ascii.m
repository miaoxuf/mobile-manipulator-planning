function occ = map_from_ascii(rows, obstacle_chars)
%MAP_FROM_ASCII Build an occupancy grid from a text drawing.
%   occ = MAP_FROM_ASCII(rows) converts a cell array of equally long
%   strings into a logical matrix.  Row k of the drawing becomes occ(k, :),
%   i.e. the first grid coordinate x indexes the text line and y the
%   character.  '#' marks an obstacle, every other character is free.
%
%   occ = MAP_FROM_ASCII(rows, chars) treats all characters in `chars`
%   as obstacles.

if nargin < 2
    obstacle_chars = '#';
end
rows = cellstr(rows);
len = cellfun(@numel, rows);
assert(all(len == len(1)), 'map_from_ascii: all rows must have the same length.');
occ = ismember(char(rows), obstacle_chars);
end
