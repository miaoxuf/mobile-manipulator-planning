function app = mm_planner_gui()
%MM_PLANNER_GUI Interactive front end of the mobile-manipulator planner.
%   MM_PLANNER_GUI() opens a window in which the robot (D-H table, joint
%   limits, dynamics), the map, the grasp targets and the energy model can
%   be edited.  Press "Plan" to run PLAN_GRASP_TASKS; all result figures
%   are shown in tabs and can be exported.

setup_paths();
cfg0 = default_config();
th = viz_theme();

app = struct();
app.cfg = cfg0;
app.out = [];
app.map = cfg0.env.map;

fig = uifigure('Name', 'Mobile manipulator planner', 'Position', [60 60 1480 900], ...
               'Color', th.surface);
main = uigridlayout(fig, [1 2], 'ColumnWidth', {'1x', 360}, 'Padding', [8 8 8 8]);

%% left: result tabs ------------------------------------------------------
tabs = uitabgroup(main);
names = {'Paths', 'Stations', 'Joint profiles', 'Energy', 'Arm poses', ...
         'Workspace', 'Convergence', 'Robot (D-H)'};
T = struct();
for k = 1:numel(names)
    T.(matlab.lang.makeValidName(names{k})) = uitab(tabs, 'Title', names{k}, ...
        'BackgroundColor', th.surface);
end

% robot table
rg = uigridlayout(T.Robot_D_H_, [3 1], 'RowHeight', {28, '1x', 28});
uilabel(rg, 'Text', ['Edit the D-H table and joint data, then press "Plan". ' ...
        'Angles in deg, lengths in mm, inertia in kg m^2, speed in rad/s.']);
dhTable = uitable(rg, 'ColumnEditable', true, 'ColumnName', ...
    {'a (mm)', 'alpha (deg)', 'd (mm)', 'offset (deg)', 'q min', 'q max', ...
     'q home', 'inertia', 'omega avg', 'alpha max'});
rb = uigridlayout(rg, [1 4], 'ColumnWidth', {'fit', 'fit', 'fit', '1x'}, 'Padding', [0 0 0 0]);
uibutton(rb, 'Text', 'Add joint', 'ButtonPushedFcn', @(~, ~) addJoint());
uibutton(rb, 'Text', 'Remove last joint', 'ButtonPushedFcn', @(~, ~) removeJoint());
convDrop = uidropdown(rb, 'Items', {'standard', 'modified'});

%% right: controls --------------------------------------------------------
ctrl = uigridlayout(main, [21 2], 'ColumnWidth', {150, '1x'}, ...
    'RowHeight', [repmat({26}, 1, 17), {'1x', 30, 30, 22}], 'RowSpacing', 6);

robotFiles = dir(fullfile(fileparts(mfilename('fullpath')), 'robots', 'robot_*.m'));
robotNames = setdiff(erase({robotFiles.name}, '.m'), {'robot_from_dh'});
row = 0;
robotDrop = addRow('Robot preset', @uidropdown, 'Items', robotNames, ...
                   'Value', 'robot_6dof_example', 'ValueChangedFcn', @(~, ~) loadRobot());
mapRow = row + 1;
uilabel(ctrl, 'Text', 'Map');
mapGrid = uigridlayout(ctrl, [1 2], 'ColumnWidth', {'1x', 'fit'}, 'Padding', [0 0 0 0]);
mapLabel = uilabel(mapGrid, 'Text', sprintf('example warehouse %dx%d', size(app.map)));
uibutton(mapGrid, 'Text', 'Load...', 'ButtonPushedFcn', @(~, ~) loadMap());
row = mapRow;
cellEdit   = addRow('Cell size (mm)', @uieditfield, 'numeric', 'Value', cfg0.env.cell_size, 'Limits', [1 Inf]);
targetEdit = addRow('Targets (x,y);...', @uieditfield, 'text', ...
                    'Value', pairsToText(cfg0.env.targets));
startEdit  = addRow('Start (x,y)', @uieditfield, 'text', 'Value', pairsToText(cfg0.env.start));
headDrop   = addRow('Start heading', @uidropdown, 'Items', {'0', '90', '180', '270', 'free'}, ...
                    'Value', '0');
heightEdit = addRow('Goods height (mm)', @uieditfield, 'numeric', 'Value', cfg0.env.target_height);
tolEdit    = addRow('Error tolerance (mm)', @uieditfield, 'numeric', 'Value', cfg0.arm.err_tol, 'Limits', [0 Inf]);
floorEdit  = addRow('Floor clearance (mm)', @uieditfield, 'numeric', 'Value', cfg0.arm.floor_clearance);
ringEdit   = addRow('Ring [rmin rmax] mm', @uieditfield, 'text', 'Value', '', ...
                    'Placeholder', 'empty = Monte Carlo');
tnomEdit   = addRow('Nominal arm time (s)', @uieditfield, 'numeric', 'Value', cfg0.arm.T_nominal, 'Limits', [0.01 Inf]);
kmEdit     = addRow('k move (J/cell)', @uieditfield, 'numeric', 'Value', cfg0.platform.k_move, 'Limits', [0 Inf]);
ktEdit     = addRow('k turn (J/deg)', @uieditfield, 'numeric', 'Value', cfg0.platform.k_turn, 'Limits', [0 Inf]);
krEdit     = addRow('k rotation (J/deg)', @uieditfield, 'numeric', 'Value', cfg0.platform.k_rot, 'Limits', [0 Inf]);
weEdit     = addRow('Weight energy', @uieditfield, 'numeric', 'Value', cfg0.select.w_energy, 'Limits', [0 Inf]);
wrEdit     = addRow('Weight error', @uieditfield, 'numeric', 'Value', cfg0.select.w_error, 'Limits', [0 Inf]);
pruneBox   = addRow('Branch and bound', @uicheckbox, 'Text', 'skip dominated stations (fast)', ...
                    'Value', cfg0.select.prune);

resTable = uitable(ctrl, 'ColumnName', {'Task', 'Station', 'Heading', 'Error (mm)', 'Energy (J)'}, ...
                   'RowName', {}, 'ColumnWidth', {40, 70, 60, 80, 'auto'});
resTable.Layout.Row = 18;  resTable.Layout.Column = [1 2];
runBtn = uibutton(ctrl, 'Text', 'Plan', 'FontWeight', 'bold', ...
                  'BackgroundColor', th.categorical(1, :), 'FontColor', [1 1 1], ...
                  'ButtonPushedFcn', @(~, ~) runPlanner());
runBtn.Layout.Row = 19;  runBtn.Layout.Column = [1 2];
expBtn = uibutton(ctrl, 'Text', 'Export figures...', 'Enable', 'off', ...
                  'ButtonPushedFcn', @(~, ~) exportFigures());
expBtn.Layout.Row = 20;  expBtn.Layout.Column = [1 2];
status = uilabel(ctrl, 'Text', 'Ready.', 'FontColor', th.ink2);
status.Layout.Row = 21;  status.Layout.Column = [1 2];

loadRobot();
drawMapPreview();

%% ------------------------------------------------------------------------
    function h = addRow(label, ctor, varargin)
        row = row + 1;
        l = uilabel(ctrl, 'Text', label);
        l.Layout.Row = row;  l.Layout.Column = 1;
        h = ctor(ctrl, varargin{:});
        h.Layout.Row = row;  h.Layout.Column = 2;
    end

    function loadRobot()
        r = feval(robotDrop.Value);
        n = size(r.dh, 1);
        dhTable.Data = [r.dh, r.qlim, r.q_home(:), r.inertia(:), r.omega_avg(:), r.alpha_max(:)];
        dhTable.RowName = arrayfun(@(i) sprintf('J%d', i), 1:n, 'UniformOutput', false);
        convDrop.Value = r.convention;
        app.robotName = r.name;
        app.robotBase = r.base_height;
    end

    function addJoint()
        d = dhTable.Data;
        d(end + 1, :) = [0 0 0 0 -180 180 0 1 1 Inf];
        dhTable.Data = d;
        dhTable.RowName = arrayfun(@(i) sprintf('J%d', i), 1:size(d, 1), 'UniformOutput', false);
    end

    function removeJoint()
        d = dhTable.Data;
        if size(d, 1) > 1
            dhTable.Data = d(1:end - 1, :);
            dhTable.RowName = dhTable.RowName(1:end - 1);
        end
    end

    function r = robotFromTable()
        d = dhTable.Data;
        r = robot_from_dh(d(:, 1:4), 'Name', app.robotName, 'Convention', convDrop.Value, ...
            'QLim', d(:, 5:6), 'QHome', d(:, 7).', 'Inertia', d(:, 8).', ...
            'OmegaAvg', d(:, 9).', 'AlphaMax', d(:, 10).', 'BaseHeight', app.robotBase);
    end

    function loadMap()
        [f, p] = uigetfile({'*.mat;*.txt', 'Map (*.mat with variable "map", *.txt ASCII)'});
        figure(fig);
        if isequal(f, 0), return, end
        try
            [~, ~, ext] = fileparts(f);
            if strcmpi(ext, '.mat')
                s = load(fullfile(p, f));
                fn = fieldnames(s);
                if isfield(s, 'map'), m = s.map; else, m = s.(fn{1}); end
            else
                m = map_from_ascii(readlines(fullfile(p, f), 'EmptyLineRule', 'skip'));
            end
            app.map = logical(m);
            mapLabel.Text = sprintf('%s (%dx%d)', f, size(app.map));
            drawMapPreview();
        catch err
            uialert(fig, err.message, 'Could not load map');
        end
    end

    function drawMapPreview()
        [tl, ~] = viz_canvas(T.Paths, 1, 1, [], '');
        ax = nexttile(tl);
        viz_style_axes(ax, 'Grid', false);
        viz_draw_map(ax, app.map, cellEdit.Value);
        title(ax, 'Map preview - press "Plan" to compute paths');
    end

    function cfg = buildConfig()
        cfg = cfg0;
        cfg.robot = robotFromTable();
        cfg.env.map = app.map;
        cfg.env.cell_size = cellEdit.Value;
        cfg.env.targets = textToPairs(targetEdit.Value);
        cfg.env.start = textToPairs(startEdit.Value);
        if strcmp(headDrop.Value, 'free')
            cfg.env.start_heading = NaN;
        else
            cfg.env.start_heading = str2double(headDrop.Value);
        end
        cfg.env.target_height = heightEdit.Value;
        cfg.arm.err_tol = tolEdit.Value;
        cfg.arm.floor_clearance = floorEdit.Value;
        cfg.arm.T_nominal = tnomEdit.Value;
        ring = str2num(ringEdit.Value); %#ok<ST2NM> numeric vector only
        if ~isempty(ring)
            assert(numel(ring) == 2 && ring(1) < ring(2), 'Ring must be [r_min r_max].');
        end
        cfg.workspace.radius = ring;
        cfg.platform.k_move = kmEdit.Value;
        cfg.platform.k_turn = ktEdit.Value;
        cfg.platform.k_rot  = krEdit.Value;
        cfg.select.w_energy = weEdit.Value;
        cfg.select.w_error  = wrEdit.Value;
        cfg.select.prune    = pruneBox.Value;
        cfg.verbose = false;
        assert(~isempty(cfg.env.targets), 'Enter at least one target, e.g. (4,12);(14,13).');
        assert(size(cfg.env.start, 1) == 1, 'Enter exactly one start cell, e.g. (1,1).');
    end

    function runPlanner()
        dlg = uiprogressdlg(fig, 'Title', 'Planning', 'Indeterminate', 'on', ...
                            'Message', 'Optimising stations, arm trajectories and paths...');
        cleanup = onCleanup(@() close(dlg));
        try
            cfg = buildConfig();
            out = plan_grasp_tasks(cfg);
        catch err
            uialert(fig, err.message, 'Planning failed');
            return
        end
        app.out = out;
        dlg.Message = 'Drawing figures...';
        fig_paths(out, T.Paths);
        fig_station_candidates(out, T.Stations);
        fig_joint_profiles(out, [], T.JointProfiles);
        fig_energy_breakdown(out, [], T.Energy);
        fig_arm_poses(out, T.ArmPoses);
        if ~isempty(out.workspace)
            fig_workspace(out, T.Workspace);
        else
            delete(T.Workspace.Children);
        end
        fig_convergence(out, T.Convergence);

        s = out.summary;
        rows = cell(height(s), 5);
        for i = 1:height(s)
            if isnan(s.station_x(i))
                rows(i, :) = {sprintf('%d', i), 'none', '-', '-', '-'};
            else
                rows(i, :) = {sprintf('%d', i), sprintf('(%g,%g)', s.station_x(i), s.station_y(i)), ...
                              sprintf('%g%s', s.orientation(i), char(176)), ...
                              sprintf('%.3f', s.error_mm(i)), sprintf('%.1f', s.E_total_J(i))};
            end
        end
        resTable.Data = rows;
        expBtn.Enable = 'on';
        status.Text = sprintf('Done in %.1f s - total energy %.1f J.', out.runtime, ...
                              sum(s.E_total_J, 'omitnan'));
        tabs.SelectedTab = T.Paths;
    end

    function exportFigures()
        d = uigetdir(cfg0.output.dir, 'Export figures to');
        figure(fig);
        if isequal(d, 0) || isempty(app.out), return, end
        oc = cfg0.output;  oc.dir = d;
        makers = {@fig_workspace, @fig_station_candidates, @fig_joint_profiles, ...
                  @fig_energy_breakdown, @fig_energy_summary, @fig_paths, ...
                  @fig_arm_poses, @fig_convergence};
        files = {'01_workspace', '02_station_candidates', '03_joint_profiles', ...
                 '04_energy_breakdown', '05_energy_summary', '06_platform_paths', ...
                 '07_arm_poses', '08_convergence'};
        for k2 = 1:numel(makers)
            if k2 == 1 && isempty(app.out.workspace), continue, end
            f = makers{k2}(app.out);
            f.Visible = 'off';
            save_figure(f, files{k2}, oc);
            close(f);
        end
        status.Text = ['Figures exported to ' d];
    end
end

% ------------------------------------------------------------------------
function P = textToPairs(s)
tok = regexp(s, '\(\s*([-+]?\d+)\s*,\s*([-+]?\d+)\s*\)', 'tokens');
P = reshape(str2double([tok{:}]), 2, []).';
end

function s = pairsToText(P)
s = strjoin(arrayfun(@(i) sprintf('(%d,%d)', P(i, 1), P(i, 2)), 1:size(P, 1), ...
            'UniformOutput', false), ';');
end
