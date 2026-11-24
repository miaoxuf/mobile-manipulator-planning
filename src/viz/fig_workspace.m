function fig = fig_workspace(out, parent)
%FIG_WORKSPACE Monte Carlo workspace and derived working ring.
%   fig = FIG_WORKSPACE(out) plots (a) the sampled reachable workspace
%   with the arm in its home pose and (b) the top view of the height band
%   used for station selection together with the working ring.
%   fig = FIG_WORKSPACE(out, parent) draws into an existing container.

if nargin < 2, parent = []; end
th = viz_theme();
robot = out.robot;
ws = out.workspace;
r = out.radius;
[tl, fig] = viz_canvas(parent, 1, 2, [1180 520], 'Workspace');
fig.UserData = struct('content', 'image');     % dense point cloud

%% (a) 3-D workspace
ax = nexttile(tl);
viz_style_axes(ax);
if ~isempty(ws)
    P = ws.points;
    k = randperm(size(P, 1), min(size(P, 1), 9000));
    P = P(k, :);
    z = P(:, 3) + robot.base_height;
    scatter3(ax, P(:, 1), P(:, 2), z, 4, z, 'filled', ...
             'MarkerFaceAlpha', 0.45, 'HandleVisibility', 'off');
    colormap(ax, interp1(linspace(0, 1, size(th.seq_blue, 1)), th.seq_blue, ...
                         linspace(0, 1, 256)));
    cb = colorbar(ax);
    cb.Label.String = 'Height z (mm)';
    cb.Color = th.ink2;
    cb.Box = 'off';
    cb.TickDirection = 'out';
end
draw_robot(ax, robot, robot.q_home, th.ink, 'LineWidth', 4);
axis(ax, 'equal');
view(ax, 135, 22);
xlabel(ax, 'x (mm)'); ylabel(ax, 'y (mm)'); zlabel(ax, 'z (mm)');
title(ax, '(a) Reachable workspace and home pose');

%% (b) top view of the height band and working ring
ax = nexttile(tl);
viz_style_axes(ax);
t = linspace(0, 2 * pi, 361);
patch(ax, [r(2) * cos(t), r(1) * cos(fliplr(t))], ...
          [r(2) * sin(t), r(1) * sin(fliplr(t))], th.categorical(1, :), ...
      'FaceAlpha', 0.12, 'EdgeColor', 'none', 'DisplayName', 'Working ring');
leg = {};
if ~isempty(ws)
    Pb = ws.points(ws.in_band, :);
    k = randperm(size(Pb, 1), min(size(Pb, 1), 6000));
    scatter(ax, Pb(k, 1), Pb(k, 2), 5, th.ink2, 'filled', 'MarkerFaceAlpha', 0.35, ...
            'DisplayName', sprintf('Samples with z in [%g, %g] mm', ws.z_band));
    leg = {'Location', 'southoutside'};
end
plot(ax, r(2) * cos(t), r(2) * sin(t), '-',  'Color', th.categorical(1, :), ...
     'LineWidth', th.line, 'HandleVisibility', 'off');
plot(ax, r(1) * cos(t), r(1) * sin(t), '--', 'Color', th.categorical(1, :), ...
     'LineWidth', th.line, 'HandleVisibility', 'off');
plot(ax, 0, 0, '+', 'Color', th.ink, 'MarkerSize', 10, 'LineWidth', 1.5, ...
     'DisplayName', 'Arm base');
a = deg2rad(40);
text(ax, r(2) * cos(a), r(2) * sin(a), sprintf('  r_{max} = %.0f mm', r(2)), ...
     'Color', th.ink, 'FontName', th.font, 'FontSize', th.font_size, ...
     'VerticalAlignment', 'bottom');
text(ax, r(1) * cos(-a), r(1) * sin(-a), sprintf('r_{min} = %.0f mm  ', r(1)), ...
     'Color', th.ink, 'FontName', th.font, 'FontSize', th.font_size, ...
     'HorizontalAlignment', 'right', 'VerticalAlignment', 'top');
axis(ax, 'equal');
lim = 1.15 * r(2);
set(ax, 'XLim', [-lim lim], 'YLim', [-lim lim]);
xlabel(ax, 'x (mm)'); ylabel(ax, 'y (mm)');
title(ax, '(b) Working ring used for station selection');
lg = legend(ax, leg{:});
lg.Box = 'off';
lg.NumColumns = 3;
lg.TextColor = th.ink2;
end
