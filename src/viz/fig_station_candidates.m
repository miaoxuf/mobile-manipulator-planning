function fig = fig_station_candidates(out, parent)
%FIG_STATION_CANDIDATES Station candidates of every task on the map.
%   Shows for each target the working ring, ring cells whose line of sight
%   is blocked, unobstructed stations, stations for which the arm reaches
%   the target within tolerance, and the selected station.

if nargin < 2, parent = []; end
th = viz_theme();
nT = numel(out.tasks);
nc = min(nT, 2);
nr = ceil(nT / nc);
[tl, fig] = viz_canvas(parent, nr, nc, [520 * nc, 500 * nr + 60], 'Station candidates');
cs = out.cfg.env.cell_size;
rc = out.radius / cs;
t  = linspace(0, 2 * pi, 241);
any_pruned = false;

for i = 1:nT
    task = out.tasks(i);
    c = viz_task_color(i);
    ax = nexttile(tl);
    viz_style_axes(ax, 'Grid', false);
    viz_draw_map(ax, out.map, cs);
    tx = task.target(1);  ty = task.target(2);

    patch(ax, tx + [rc(2) * cos(t), rc(1) * cos(fliplr(t))], ...
              ty + [rc(2) * sin(t), rc(1) * sin(fliplr(t))], c, ...
          'FaceAlpha', 0.10, 'EdgeColor', 'none');
    for rr = rc
        plot(ax, tx + rr * cos(t), ty + rr * sin(t), '-', 'Color', [c 0.6], 'LineWidth', 0.8);
    end

    cand = task.candidates;
    B = cand.ring(cand.blocked, :);
    plot(ax, B(:, 1), B(:, 2), 'x', 'Color', th.muted, 'MarkerSize', 6, 'LineWidth', 1.2);

    V = cand.valid;
    ev = task.evals;
    F = unique([ev.station_x(ev.feasible), ev.station_y(ev.feasible)], 'rows');
    Ev = unique([ev.station_x(ev.evaluated), ev.station_y(ev.evaluated)], 'rows');
    isF = ismember(V, F, 'rows');
    isE = ismember(V, Ev, 'rows');
    isP = ~isE;                                   % pruned: never optimised
    any_pruned = any_pruned || any(isP);
    plot(ax, V(isP, 1), V(isP, 2), 'o', 'Color', th.muted, 'MarkerSize', 6, 'LineWidth', 1);
    plot(ax, V(isE & ~isF, 1), V(isE & ~isF, 2), 'o', 'Color', c, 'MarkerSize', 7, 'LineWidth', 1.4);
    plot(ax, V(isF, 1), V(isF, 2), 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', th.surface, ...
         'MarkerSize', 8, 'LineWidth', 1);

    if task.found
        s = task.best.station;
        plot(ax, [s(1) tx], [s(2) ty], '--', 'Color', th.ink, 'LineWidth', 1);
        plot(ax, s(1), s(2), 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', th.ink, ...
             'MarkerSize', 12, 'LineWidth', 1.8);
    end
    plot(ax, tx, ty, 's', 'MarkerFaceColor', c, 'MarkerEdgeColor', th.ink, ...
         'MarkerSize', 10, 'LineWidth', 1.4);
    text(ax, tx + 0.45, ty + 0.45, sprintf('T%d', i), 'Color', th.ink, ...
         'FontName', th.font, 'FontSize', th.font_size, 'FontWeight', 'bold');
    title(ax, sprintf('Task %d: target (%d, %d) | %d unobstructed, %d optimised, %d feasible', ...
          i, tx, ty, task.n_valid, size(Ev, 1), task.n_feasible));
end

% shared legend drawn with neutral proxies
ax = nexttile(tl, 1);
g = th.ink2;
h = [plot(ax, nan, nan, 's', 'MarkerFaceColor', g, 'MarkerEdgeColor', th.ink, 'MarkerSize', 9)
     patch(ax, nan, nan, g, 'FaceAlpha', 0.15, 'EdgeColor', g)
     plot(ax, nan, nan, 'x', 'Color', th.muted, 'MarkerSize', 7, 'LineWidth', 1.2)
     plot(ax, nan, nan, 'o', 'Color', th.muted, 'MarkerSize', 6, 'LineWidth', 1)
     plot(ax, nan, nan, 'o', 'Color', g, 'MarkerSize', 7, 'LineWidth', 1.4)
     plot(ax, nan, nan, 'o', 'MarkerFaceColor', g, 'MarkerEdgeColor', th.surface, 'MarkerSize', 8)
     plot(ax, nan, nan, 'o', 'MarkerFaceColor', g, 'MarkerEdgeColor', th.ink, 'MarkerSize', 11, 'LineWidth', 1.8)];
names = {'Target', 'Working ring', 'Line of sight blocked', 'Pruned (not optimised)', ...
         'Optimised, infeasible', 'Arm feasible station', 'Selected station'};
keep = true(1, numel(names));
keep(4) = any_pruned;
lg = legend(ax, h(keep), names(keep), 'Orientation', 'horizontal', 'NumColumns', 4);
lg.Layout.Tile = 'south';
lg.Box = 'off';
lg.TextColor = th.ink2;
lg.FontSize = th.font_size;
end
