function fig = fig_energy_breakdown(out, top_k, parent)
%FIG_ENERGY_BREAKDOWN Energy and error of candidate stations x headings.
%   fig = FIG_ENERGY_BREAKDOWN(out) shows, per task, the stacked energy
%   (platform moving / turning / rotation + manipulator) of every platform
%   heading at the best `top_k` stations (default 4), and below it the
%   grasping error on a separate, logarithmic axis.  The selected
%   combination is outlined.

if nargin < 2 || isempty(top_k), top_k = 4; end
if nargin < 3, parent = []; end
th = viz_theme();
nT = numel(out.tasks);
orients = out.cfg.platform.orientations;
nO = numel(orients);
tol = out.cfg.arm.err_tol;
[tl, fig] = viz_canvas(parent, 4, nT, [max(380 * nT, 600), 680], 'Energy breakdown');
gap = 1.3;
hb = gobjects(0);
hpr = gobjects(0);

for i = 1:nT
    task = out.tasks(i);
    axE = nexttile(tl, i, [3 1]);
    viz_style_axes(axE);
    axR = nexttile(tl, 3 * nT + i);
    viz_style_axes(axR);
    title(axE, sprintf('Task %d: target (%d, %d)', i, task.target));
    ev = task.evals;
    if ~task.found
        text(axE, 0.5, 0.5, 'no feasible station', 'Units', 'normalized', ...
             'HorizontalAlignment', 'center', 'Color', th.ink2);
        continue
    end

    % rank stations by their best score; combinations skipped by branch
    % and bound are ranked by their lower bound (platform energy)
    [S, ~, g] = unique([ev.station_x, ev.station_y], 'rows', 'stable');
    key = ev.score;
    pr = ~ev.evaluated & isfinite(ev.E_platform);
    key(pr) = out.cfg.select.w_energy * ev.E_platform(pr);
    best_score = accumarray(g, key, [], @min);
    [best_score, order] = sort(best_score);
    order = order(isfinite(best_score));
    order = order(1:min(top_k, numel(order)));
    nS = numel(order);

    X = zeros(nS * nO, 1); Y = zeros(nS * nO, 4); E = nan(nS * nO, 1);
    F = false(nS * nO, 1); Pr = false(nS * nO, 1); Ep = nan(nS * nO, 1); sel = 0;
    for a = 1:nS
        for o = 1:nO
            k = (a - 1) * nO + o;
            X(k) = (a - 1) * (nO + gap) + o;
            row = find(g == order(a) & ev.orientation == orients(o), 1);
            if isempty(row), continue, end
            E(k) = ev.error(row);
            F(k) = ev.feasible(row);
            Pr(k) = pr(row);
            Ep(k) = ev.E_platform(row);
            if F(k)
                Y(k, :) = [ev.E_move(row), ev.E_turn(row), ev.E_rot(row), ev.E_arm(row)];
            end
            if row == task.best.row, sel = k; end
        end
    end

    b = bar(axE, X, Y, 0.78, 'stacked', 'EdgeColor', th.surface, 'LineWidth', 0.8);
    for c = 1:4
        b(c).FaceColor = th.energy(c, :);
    end
    hb = b;
    for k = find(Pr).'
        rectangle(axE, 'Position', [X(k) - 0.39, 0, 0.78, Ep(k)], ...
                  'EdgeColor', th.muted, 'LineStyle', '--', 'LineWidth', 1);
    end
    if any(Pr) && isempty(hpr)
        hpr = patch(axE, nan, nan, 'w', 'FaceColor', 'none', 'EdgeColor', th.muted, ...
                    'LineStyle', '--', 'LineWidth', 1);
    end
    bad = ~F & ~Pr;
    plot(axE, X(bad), zeros(nnz(bad), 1), 'x', 'Color', th.muted, ...
         'MarkerSize', 7, 'LineWidth', 1.2);
    if sel > 0
        tot = sum(Y(sel, :));
        rectangle(axE, 'Position', [X(sel) - 0.45, 0, 0.9, tot], ...
                  'EdgeColor', th.ink, 'LineWidth', 1.6);
        text(axE, X(sel), tot, sprintf('%.1f J', tot), 'Color', th.ink, ...
             'FontName', th.font, 'FontSize', th.font_size, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    end
    centers = (0:nS - 1) * (nO + gap) + (nO + 1) / 2;
    xl = [0.3, X(end) + 0.7];
    set(axE, 'XLim', xl, 'XTick', centers, ...
        'XTickLabel', arrayfun(@(a) sprintf('(%d,%d)', S(order(a), :)), 1:nS, ...
                               'UniformOutput', false));
    ylim(axE, [0, 1.12 * max([sum(Y, 2); Ep(Pr)]) + eps]);
    axE.XGrid = 'off';
    if i == 1, ylabel(axE, 'Energy (J)'); end

    % error panel
    ok = F & isfinite(E);
    bad = ~F & isfinite(E);
    yline(axR, tol, '--', sprintf('tolerance %g mm', tol), 'Color', th.ink2, ...
          'LineWidth', 1, 'FontSize', th.font_size - 1, 'LabelHorizontalAlignment', 'left');
    plot(axR, X(ok), max(E(ok), 1e-3), 'o', 'MarkerFaceColor', viz_task_color(i), ...
         'MarkerEdgeColor', th.surface, 'MarkerSize', 7);
    plot(axR, X(bad), E(bad), 'o', 'Color', th.muted, 'MarkerSize', 6, 'LineWidth', 1.2);
    Ev = [E(isfinite(E)); tol];
    ylo = 10 ^ floor(log10(max(min(Ev), 1e-4)));
    yhi = 10 ^ ceil(log10(max(Ev) * 3));
    dec = log10(ylo):log10(yhi);
    dec = dec(1:max(1, ceil(numel(dec) / 4)):end);
    set(axR, 'YScale', 'log', 'XLim', xl, 'YLim', [ylo yhi], 'YTick', 10 .^ dec, ...
        'YMinorGrid', 'off', 'XTick', X, ...
        'XTickLabel', repmat(arrayfun(@(o) sprintf('%g', o), orients, ...
                                      'UniformOutput', false), 1, nS));
    axR.XTickLabelRotation = 90;
    axR.FontSize = th.font_size - 2;
    axR.YLabel.FontSize = th.font_size;
    axR.XGrid = 'off';
    if i == 1, ylabel(axR, 'Error (mm)'); end
end

if ~isempty(hb)
    hl = hb(:).';  names = th.energy_names;
    if ~isempty(hpr)
        hl(end + 1) = hpr;
        names{end + 1} = 'Pruned: platform energy (lower bound)';
    end
    lg = legend(hl, names, 'Orientation', 'horizontal', 'NumColumns', numel(hl));
    lg.Layout.Tile = 'north';
    lg.Box = 'off';
    lg.TextColor = th.ink2;
end
xlabel(tl, sprintf('Candidate station (x, y); bars and dots per station: platform heading (%s)', ...
       char(176)), 'Color', th.ink2, ...
       'FontName', th.font, 'FontSize', th.font_size);
end
