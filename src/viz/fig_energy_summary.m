function fig = fig_energy_summary(out, parent)
%FIG_ENERGY_SUMMARY Energy composition of the selected plan of every task.

if nargin < 2, parent = []; end
th = viz_theme();
[tl, fig] = viz_canvas(parent, 1, 1, [760 520], 'Energy summary');
ax = nexttile(tl);
viz_style_axes(ax);

tasks = out.tasks([out.tasks.found]);
ids = find([out.tasks.found]);
n = numel(tasks);
Y = zeros(n, 4);
for i = 1:n
    b = tasks(i).best;
    Y(i, :) = [b.E_move, b.E_turn, b.E_rot, b.E_arm];
end
if n == 0
    return
end
bh = bar(ax, 1:n, Y, 0.55, 'stacked', 'EdgeColor', th.surface, 'LineWidth', 1);
for c = 1:4
    bh(c).FaceColor = th.energy(c, :);
end
tot = sum(Y, 2);
for i = 1:n
    text(ax, i, tot(i), {sprintf('%.1f J', tot(i)), ...
         sprintf('error %.2f mm', tasks(i).best.error)}, ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
         'Color', th.ink, 'FontName', th.font, 'FontSize', th.font_size);
end
labels = arrayfun(@(i) sprintf('Task %d (%d,%d)', ids(i), tasks(i).target), 1:n, ...
                  'UniformOutput', false);
set(ax, 'XTick', 1:n, 'XTickLabel', labels, 'XLim', [0.4, n + 0.6]);
ax.XGrid = 'off';
ylim(ax, [0, 1.22 * max(tot)]);
ylabel(ax, 'Energy (J)');
title(ax, sprintf('Selected plans: total energy %.1f J, total error %.2f mm', ...
      sum(tot), sum(arrayfun(@(t) t.best.error, tasks))));
lg = legend(bh, th.energy_names, 'Orientation', 'horizontal', 'NumColumns', 4);
lg.Layout.Tile = 'north';
lg.Box = 'off';
lg.TextColor = th.ink2;
end
