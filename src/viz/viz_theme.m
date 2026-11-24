function th = viz_theme()
%VIZ_THEME Colours, fonts and line widths shared by all figures.
%   The categorical palette is a colour-vision-deficiency checked order
%   (blue, orange, aqua, yellow, magenta, green, violet, red); slots are
%   always assigned in this fixed order.

hex = @(h) sscanf(h(2:end), '%2x%2x%2x', [1 3]) / 255;

th.categorical = cell2mat(cellfun(hex, { ...
    '#2a78d6', '#eb6834', '#1baf7a', '#eda100', ...
    '#e87ba4', '#008300', '#6250d6', '#e34948'}, 'UniformOutput', false).');

th.task    = th.categorical(1:4, :);          % one colour per grasp task
th.joint   = th.categorical(1:6, :);          % one colour per joint
th.energy  = th.categorical([3 4 5 7], :);    % move, turn, rotation, arm
th.energy_names = {'Platform: moving', 'Platform: turning', ...
                   'Platform: rotation', 'Manipulator'};
th.task_markers = {'o', '^', 'd', 'v', 's', 'p'};

% sequential blue ramp (light -> dark)
th.seq_blue = cell2mat(cellfun(hex, { ...
    '#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95', '#0d366b'}, ...
    'UniformOutput', false).');

th.surface  = hex('#fcfcfb');
th.ink      = hex('#0b0b0b');
th.ink2     = hex('#52514e');
th.muted    = hex('#8c8b86');
th.grid     = hex('#e4e3de');
th.obstacle = hex('#5b5a56');
th.free     = hex('#fcfcfb');

th.font      = 'Arial';
th.font_size = 10;
th.title_size = 11;
th.line      = 1.6;
th.thin      = 0.8;
th.marker    = 7;
end
