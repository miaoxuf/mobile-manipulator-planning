function fig = fig_arm_poses(out, parent)
%FIG_ARM_POSES 3-D view of the home pose and the optimised grasp pose.
%   One tile per task, drawn in the arm base frame of the selected
%   station: home pose (grey), grasp pose (task colour), end-effector path
%   of the quintic trajectory (dotted) and the goods (black cross).

if nargin < 2, parent = []; end
th = viz_theme();
ids = find([out.tasks.found]);
nT = max(numel(ids), 1);
nc = min(nT, 2);
nr = ceil(nT / nc);
[tl, fig] = viz_canvas(parent, nr, nc, [560 * nc, 470 * nr + 50], 'Arm poses');
fig.UserData = struct('content', 'image');     % transparent 3-D scene
robot = out.robot;
R = 1.1 * out.radius(2);

for k = 1:numel(ids)
    task = out.tasks(ids(k));
    b = task.best;
    c = viz_task_color(ids(k));
    ax = nexttile(tl);
    viz_style_axes(ax);
    patch(ax, [-R R R -R], [-R -R R R], [0 0 0 0], th.grid, 'FaceAlpha', 0.35, ...
          'EdgeColor', 'none', 'HandleVisibility', 'off', 'Clipping', 'on');
    h1 = draw_robot(ax, robot, robot.q_home, th.muted, 'LineWidth', 3, 'Alpha', 0.7);
    h2 = draw_robot(ax, robot, b.q, c, 'LineWidth', 5);
    ee = b.traj.ee;
    h3 = plot3(ax, ee(:, 1), ee(:, 2), ee(:, 3) + robot.base_height, ':', ...
               'Color', th.ink2, 'LineWidth', 1.6);
    tg = b.target_local;
    h4 = plot3(ax, tg(1), tg(2), tg(3) + robot.base_height, 'x', 'Color', th.ink, ...
               'MarkerSize', 12, 'LineWidth', 2.2);
    pts = [fk_chain(robot, robot.q_home); fk_chain(robot, b.q); ee; tg];
    pts(:, 3) = pts(:, 3) + robot.base_height;
    lo = min(pts, [], 1) - 250;
    hi = max(pts, [], 1) + 250;
    lo(3) = 0;
    axis(ax, 'equal');
    set(ax, 'XLim', [lo(1) hi(1)], 'YLim', [lo(2) hi(2)], 'ZLim', [lo(3) hi(3)]);
    view(ax, 125, 28);
    xlabel(ax, 'x (mm)'); ylabel(ax, 'y (mm)'); zlabel(ax, 'z (mm)');
    title(ax, sprintf('Task %d: station (%d,%d), %g%s, error %.2f mm', ids(k), ...
          b.station, b.orientation, char(176), b.error));
    if k == 1
        lg = legend(ax, [h1 h2 h3 h4], {'Home pose', 'Grasp pose', ...
                    'End-effector path', 'Goods'}, 'Orientation', 'horizontal', ...
                    'NumColumns', 4);
        lg.Layout.Tile = 'south';
        lg.Box = 'off';
        lg.TextColor = th.ink2;
    end
end
end
