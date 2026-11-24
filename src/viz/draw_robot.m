function h = draw_robot(ax, robot, q, color, varargin)
%DRAW_ROBOT Draw a stick model of the arm in its base frame.
%   h = DRAW_ROBOT(ax, robot, q, color) draws links as thick lines and
%   joints as ringed dots.  Options: 'LineWidth' (5), 'Alpha' (1),
%   'Joints' (true).

p = inputParser;
p.addParameter('LineWidth', 5);
p.addParameter('Alpha', 1);
p.addParameter('Joints', true);
p.parse(varargin{:});
th = viz_theme();

J = fk_chain(robot, q);
J = [J(1, :) - [0 0 robot.base_height]; J];      % pedestal from floor
J(:, 3) = J(:, 3) + robot.base_height;
c = [color, p.Results.Alpha];
h = plot3(ax, J(:, 1), J(:, 2), J(:, 3), '-', 'Color', c, ...
          'LineWidth', p.Results.LineWidth);
if p.Results.Joints
    scatter3(ax, J(2:end, 1), J(2:end, 2), J(2:end, 3), 36, color, 'filled', ...
             'MarkerEdgeColor', th.surface, 'LineWidth', 1.5, ...
             'MarkerFaceAlpha', p.Results.Alpha, 'HandleVisibility', 'off');
end
end
