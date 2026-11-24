function fig = fig_paths(out, parent)
%FIG_PATHS Minimum-energy platform paths, stations and headings.
%   Every task is drawn in its own colour with its own station marker;
%   arrows show the platform heading at the station and dashed lines the
%   reach from the station to the goods.

if nargin < 2, parent = []; end
th = viz_theme();
[tl, fig] = viz_canvas(parent, 1, 1, [860 860], 'Platform paths');
ax = nexttile(tl);
viz_style_axes(ax, 'Grid', false);
viz_draw_map(ax, out.map, out.cfg.env.cell_size);

nT = numel(out.tasks);
h = gobjects(0);
labels = {};
for i = 1:nT
    task = out.tasks(i);
    c = viz_task_color(i);
    mk = th.task_markers{mod(i - 1, numel(th.task_markers)) + 1};
    tx = task.target(1);  ty = task.target(2);
    if task.found
        b = task.best;
        off = (i - (nT + 1) / 2) * 0.09;          % separate overlapping paths
        P = b.path.path + off;
        h(end + 1) = plot(ax, P(:, 1), P(:, 2), '-', 'Color', c, 'LineWidth', 2.4); %#ok<AGROW>
        s = b.station;
        plot(ax, [s(1) tx], [s(2) ty], ':', 'Color', c, 'LineWidth', 1.6);
        quiver(ax, s(1), s(2), 0.9 * cosd(b.orientation), 0.9 * sind(b.orientation), 0, ...
               'Color', th.ink, 'LineWidth', 1.5, 'MaxHeadSize', 1.2);
        plot(ax, s(1), s(2), mk, 'MarkerFaceColor', c, 'MarkerEdgeColor', th.surface, ...
             'MarkerSize', 11, 'LineWidth', 1.5);
        text(ax, s(1) - 0.35, s(2) - 0.45, sprintf('S%d', i), 'Color', th.ink, ...
             'FontName', th.font, 'FontSize', th.font_size, 'HorizontalAlignment', 'right');
        labels{end + 1} = sprintf('Task %d: station (%d,%d), %g%s, %.1f J, %.2f mm', ...
            i, s, b.orientation, char(176), b.E_total, b.error); %#ok<AGROW>
    end
    plot(ax, tx, ty, 's', 'MarkerFaceColor', c, 'MarkerEdgeColor', th.ink, ...
         'MarkerSize', 11, 'LineWidth', 1.4);
    text(ax, tx + 0.45, ty + 0.45, sprintf('T%d', i), 'Color', th.ink, ...
         'FontName', th.font, 'FontSize', th.font_size, 'FontWeight', 'bold');
end
st = out.cfg.env.start;
plot(ax, st(1), st(2), 'o', 'MarkerFaceColor', th.ink, 'MarkerEdgeColor', th.surface, ...
     'MarkerSize', 12, 'LineWidth', 1.5);
text(ax, st(1) + 0.5, st(2) - 0.1, 'Start', 'Color', th.ink, 'FontName', th.font, ...
     'FontSize', th.font_size, 'FontWeight', 'bold', 'VerticalAlignment', 'top');
title(ax, 'Minimum-energy paths of the mobile platform');

if ~isempty(h)
    lg = legend(ax, h, labels, 'NumColumns', 2);
    lg.Layout.Tile = 'south';
    lg.Box = 'off';
    lg.TextColor = th.ink2;
end
end
