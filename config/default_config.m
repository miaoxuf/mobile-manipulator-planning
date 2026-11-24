function cfg = default_config()
%DEFAULT_CONFIG Default parameters of the mobile-manipulator planner.
%   cfg = DEFAULT_CONFIG() returns a struct that fully describes one
%   planning run.  Copy this file (or modify the returned struct) to set
%   up your own robot, warehouse layout, tasks and energy coefficients.
%
%   Units: length in mm, angles in degrees (joint angles, headings),
%   time in s, energy in J.  Grid coordinates are 1-based cell indices
%   (x, y) that address the occupancy map as map(x, y).

%% General ----------------------------------------------------------------
cfg.seed    = 2024;         % random seed (Monte Carlo workspace + GA)
cfg.verbose = true;         % print progress to the command window

%% Manipulator ------------------------------------------------------------
% Any serial arm described by D-H parameters can be used, see
% robots/robot_6dof_example.m and robots/robot_from_dh.m.
cfg.robot = robot_6dof_example();

%% Environment ------------------------------------------------------------
cfg.env.map           = warehouse_map();   % logical occupancy grid, true = obstacle
cfg.env.cell_size     = 300;               % edge length of one grid cell (mm)
cfg.env.start         = [1 1];             % start cell of the mobile platform
cfg.env.start_heading = 0;                 % initial heading (deg); NaN = free
cfg.env.targets       = [4 12; 14 13; 12 3; 1 19];   % goods positions (cells)
cfg.env.target_height = 200;               % height of the goods above floor (mm)

%% Workspace of the manipulator -------------------------------------------
% The admissible station ring around a target is derived from the arm
% workspace.  Leave `radius` empty to estimate it by Monte Carlo sampling
% of the current robot, or set [r_min r_max] (mm) explicitly.
cfg.workspace.radius      = [];
cfg.workspace.num_samples = 50000;          % Monte Carlo samples
cfg.workspace.z_band      = [];             % [z_lo z_hi] in base frame (mm);
                                            % empty = target height + [-100 200]
cfg.workspace.margin      = 0;              % shrink ring on both sides (mm)

%% Manipulator trajectory optimisation ------------------------------------
cfg.arm.err_tol   = 10;      % admissible grasping position error (mm)
cfg.arm.w_energy  = 1;       % weight of arm energy in the arm objective
cfg.arm.w_error   = 1;       % weight of position error (per mm)
cfg.arm.penalty   = 10;      % quadratic penalty for error beyond err_tol
cfg.arm.floor_clearance = 50; % joints of the grasp pose stay this high above
                              % the floor (mm); -Inf disables the constraint
cfg.arm.T_nominal = 1.5;     % nominal motion time (s); stretched if a joint
                             % would exceed its average speed / acc. limit
cfg.arm.dt        = 0.01;    % sampling step of the reported trajectory (s)
cfg.arm.solver    = @solver_idp_ga;   % pluggable solver, see solver_idp_ga.m

cfg.arm.warm_start = false;     % seed the solver with earlier solutions of
                                % nearby targets (helps large exhaustive runs;
                                % little gain together with branch and bound)
cfg.arm.warm_k     = 2;         % number of warm-start seeds

cfg.arm.idp.starts      = 6;    % cold start: home pose + (starts-1) random poses
cfg.arm.idp.starts_warm = 2;    % random starts added to the warm-start seeds
cfg.arm.idp.grid      = 41;     % candidate angles per joint and sweep
cfg.arm.idp.max_iter  = 25;     % IDP iterations (full sweeps over all joints)
cfg.arm.idp.shrink    = 0.7;    % region contraction factor per iteration
cfg.arm.idp.min_range = 0.01;   % stop when the search region is below (deg)

cfg.arm.ga.pop_size       = 60;
cfg.arm.ga.generations    = 120;
cfg.arm.ga.crossover_rate = 0.9;
cfg.arm.ga.mutation_rate  = 0.2;   % per-gene probability
cfg.arm.ga.mutation_scale = 0.1;   % std as a fraction of the joint range
cfg.arm.ga.elite          = 2;
cfg.arm.ga.tournament     = 3;
cfg.arm.ga.stall_generations = 25; % stop after this many generations without
                                   % improvement (0 = run all generations)

cfg.arm.polish = true;           % short local IDP refinement after the GA

cfg.arm.nullspace.enable   = true;  % gradient refinement along the IK manifold
cfg.arm.nullspace.max_iter = 40;
cfg.arm.nullspace.step     = 5;     % max null-space step per iteration (deg)

%% Mobile platform energy model -------------------------------------------
cfg.platform.k_move       = 1;     % J per grid step
cfg.platform.k_turn       = 0.5;   % J per degree of heading change en route
cfg.platform.k_rot        = 0.3;   % J per degree of in-place rotation at station
cfg.platform.orientations = [0 90 180 270];   % admissible station headings (deg)

%% Station / orientation selection ----------------------------------------
cfg.select.w_energy = 1;   % weight of total energy (J)
cfg.select.w_error  = 1;   % weight of grasping error (mm)
cfg.select.prune    = true; % exact branch and bound: skip combinations whose
                            % platform energy alone exceeds the best score.
                            % false = optimise every station x heading
                            % (full comparison in the energy figure)

%% Output -----------------------------------------------------------------
cfg.output.save    = true;
cfg.output.dir     = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
cfg.output.formats = {'png', 'pdf'};
cfg.output.dpi     = 300;
end
