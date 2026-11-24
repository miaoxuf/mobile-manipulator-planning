function fig = fig_convergence(out, parent)
%FIG_CONVERGENCE Convergence of the arm optimiser for the selected plans.
%   Four small multiples, one per solver stage: multi-start IDP sweeps and
%   GA generations share a logarithmic cost axis (the penalty dominates
%   early on); the local IDP polish and the null-space refinement use
%   their own linear axes because their improvements are small.

if nargin < 2, parent = []; end
th = viz_theme();
ids = find([out.tasks.found]);
stages = {'idp', 'ga', 'polish', 'nullspace'};
names  = {'Iterative dynamic programming', 'Genetic algorithm', ...
          'Local IDP refinement', 'Null-space refinement'};
xl     = {'Sweep', 'Generation', 'Sweep', 'Iteration'};
[tl, fig] = viz_canvas(parent, 1, 4, [1400 420], 'Convergence');

h = gobjects(numel(ids), 1);
axs = gobjects(1, 4);
for s = 1:4
    ax = nexttile(tl);
    axs(s) = ax;
    viz_style_axes(ax);
    any_data = false;
    for k = 1:numel(ids)
        hist = out.tasks(ids(k)).best.history;
        if ~isfield(hist, stages{s}) || isempty(hist.(stages{s}))
            continue
        end
        y = hist.(stages{s});
        x = 1:numel(y);
        if strcmp(stages{s}, 'nullspace'), x = x - 1; end   % iteration 0 = start
        h(k) = plot(ax, x, y, '-', 'Color', viz_task_color(ids(k)), ...
                    'LineWidth', th.line, 'Marker', '.', 'MarkerSize', 8);
        any_data = true;
    end
    if s <= 2
        set(ax, 'YScale', 'log');
    end
    if ~any_data
        text(ax, 0.5, 0.5, 'stage disabled', 'Units', 'normalized', ...
             'HorizontalAlignment', 'center', 'Color', th.ink2);
    end
    title(ax, names{s});
    xlabel(ax, xl{s});
    if s == 1, ylabel(ax, 'Objective (weighted energy + error)'); end
end
linkaxes(axs(1:2), 'y');
valid = isgraphics(h);
if any(valid)
    lg = legend(h(valid), arrayfun(@(i) sprintf('Task %d', i), ids(valid), ...
                'UniformOutput', false), 'Orientation', 'horizontal');
    lg.Layout.Tile = 'north';
    lg.Box = 'off';
    lg.TextColor = th.ink2;
end
end
