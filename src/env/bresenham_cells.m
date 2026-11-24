function cells = bresenham_cells(p0, p1)
%BRESENHAM_CELLS Grid cells on the discrete line between two cells.
%   cells = BRESENHAM_CELLS(p0, p1) returns a K-by-2 list of integer
%   cells [x y] from p0 to p1 (both included) using Bresenham's
%   algorithm, valid for all octants.

x0 = round(p0(1)); y0 = round(p0(2));
x1 = round(p1(1)); y1 = round(p1(2));
dx =  abs(x1 - x0);  sx = sign(x1 - x0);
dy = -abs(y1 - y0);  sy = sign(y1 - y0);
err = dx + dy;

cells = zeros(max(dx, -dy) + 1, 2);
k = 0;
while true
    k = k + 1;
    cells(k, :) = [x0 y0];
    if x0 == x1 && y0 == y1
        break
    end
    e2 = 2 * err;
    if e2 >= dy          % step in x
        err = err + dy;
        x0  = x0 + sx;
    end
    if e2 <= dx          % step in y
        err = err + dx;
        y0  = y0 + sy;
    end
end
cells = cells(1:k, :);
end
