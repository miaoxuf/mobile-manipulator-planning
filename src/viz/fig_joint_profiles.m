function fig = fig_joint_profiles(out, ids, parent)
%FIG_JOINT_PROFILES Joint angle, velocity and acceleration of the grasps.
%   fig = FIG_JOINT_PROFILES(out) draws one column per task (small
%   multiples) with the quintic profiles of the selected solution.
%   fig = FIG_JOINT_PROFILES(out, ids) restricts the plot to tasks `ids`.

if nargin < 2 || isempty(ids), ids = find([out.tasks.found]); end
if nargin < 3, parent = []; end
th = viz_theme();
ids = ids([out.tasks(ids).found]);
nc = max(numel(ids), 1);
[tl, fig] = viz_canvas(parent, 3, nc, [330 * nc + 120, 760], 'Joint profiles');
if isempty(ids)
    return
end
n = size(out.robot.dh, 1);
jc = @(j) th.categorical(mod(j - 1, size(th.categorical, 1)) + 1, :);
fields = {'q', 'qd', 'qdd'};
ylab = {'Angle (deg)', 'Velocity (deg/s)', ['Acceleration (deg/s' char(178) ')']};

h = gobjects(n, 1);
for r = 1:3
    for k = 1:nc
        task = out.tasks(ids(k));
        tr = task.best.traj;
        ax = nexttile(tl, (r - 1) * nc + k);
        viz_style_axes(ax);
        Y = tr.(fields{r});
        for j = 1:n
            h(j) = plot(ax, tr.t, Y(:, j), '-', 'Color', jc(j), 'LineWidth', th.line);
        end
        xlim(ax, [0 tr.T]);
        if r == 1
            b = task.best;
            title(ax, {sprintf('Task %d: station (%d,%d), %g%s', ids(k), b.station, ...
                  b.orientation, char(176)), ...
                  sprintf('arm energy %.2f J, duration %.2f s', tr.energy, tr.T)}, ...
                  'Interpreter', 'none');
        end
        if k == 1, ylabel(ax, ylab{r}); end
        if r == 3, xlabel(ax, 'Time (s)'); end
    end
end
lg = legend(h, arrayfun(@(j) sprintf('Joint %d', j), 1:n, 'UniformOutput', false), ...
            'Orientation', 'horizontal', 'NumColumns', n);
lg.Layout.Tile = 'north';
lg.Box = 'off';
lg.TextColor = th.ink2;
end
