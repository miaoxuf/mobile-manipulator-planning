function viz_draw_map(ax, occ, cell_size)
%VIZ_DRAW_MAP Draw an occupancy grid with x to the right and y upwards.
%   VIZ_DRAW_MAP(ax, occ, cell_size) renders free cells, obstacles and thin
%   cell borders.  Cell (x, y) is centred at the data point (x, y).
%   If cell_size is given, a secondary label shows the metric size.

th = viz_theme();
[R, C] = size(occ);

rgb = repmat(reshape(th.free, 1, 1, 3), C, R);
mask = occ.';                                    % image rows = y, columns = x
for c = 1:3
    layer = rgb(:, :, c);
    layer(mask) = th.obstacle(c);
    rgb(:, :, c) = layer;
end
image(ax, 'XData', [1 R], 'YData', [1 C], 'CData', rgb);

% cell borders
xs = 0.5:1:R + 0.5;  ys = 0.5:1:C + 0.5;
lx = [repmat(xs, 2, 1); nan(1, numel(xs))];
ly = [repmat([0.5; C + 0.5], 1, numel(xs)); nan(1, numel(xs))];
mx = [repmat([0.5; R + 0.5], 1, numel(ys)); nan(1, numel(ys))];
my = [repmat(ys, 2, 1); nan(1, numel(ys))];
plot(ax, [lx(:); mx(:)], [ly(:); my(:)], '-', 'Color', th.grid, ...
     'LineWidth', 0.5, 'HandleVisibility', 'off');
rectangle(ax, 'Position', [0.5 0.5 R C], 'EdgeColor', th.ink2, 'LineWidth', th.thin);

axis(ax, 'equal');
set(ax, 'YDir', 'normal', 'XLim', [0.5 R + 0.5], 'YLim', [0.5 C + 0.5], ...
        'XTick', unique([1, 5:5:R, R]), 'YTick', unique([1, 5:5:C, C]));
grid(ax, 'off');
xlabel(ax, 'x (cell)');
ylabel(ax, 'y (cell)');
if nargin > 2 && ~isempty(cell_size)
    ax.XLabel.String = sprintf('x (cell, %g mm)', cell_size);
    ax.YLabel.String = sprintf('y (cell, %g mm)', cell_size);
end
end
