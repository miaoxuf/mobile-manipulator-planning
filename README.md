# Mobile Manipulator Planning

English | [中文](README_zh.md)

Energy- and accuracy-aware path planning for a mobile manipulator grasping goods in a grid warehouse, implemented in MATLAB. This is the code of our paper *Path planning for mobile manipulator based on energy consumption and grasping accuracy constraints*.

## Features

- Works with any serial arm given by a D-H table (standard / modified)
- Station selection from the arm workspace and line of sight to the goods
- Arm trajectory optimisation: iterative dynamic programming + genetic algorithm + null-space refinement
- Minimum-energy platform paths with a heading-aware A* / Dijkstra
- Exact branch and bound over stations and headings, sub-second planning
- Publication-ready figures and an interactive GUI; no toolbox required

## Requirements

MATLAB R2020b or newer.

## Quick Start

```matlab
setup_paths
out = run_demo;      % plan the example tasks, figures are saved to ./results
mm_planner_gui       % interactive GUI
run_tests;           % unit tests
```

## Usage

All parameters are in [`config/default_config.m`](config/default_config.m).

```matlab
cfg = default_config();

% robot: D-H table [a alpha d offset] (mm, deg) and joint data
cfg.robot = robot_from_dh(dh, 'QLim', qlim, 'QHome', q0, ...
                          'Inertia', I, 'OmegaAvg', w);

% map: '#' = obstacle, one character = one cell
cfg.env.map = map_from_ascii({'..........'; '.###..###.'; '..........'});
cfg.env.cell_size = 300;              % mm
cfg.env.start     = [1 1];
cfg.env.targets   = [3 9; 1 10];

cfg.arm.err_tol   = 10;               % grasping error tolerance (mm)
out = run_demo(cfg);
```

The arm solver can be replaced through `cfg.arm.solver` (see [`solver_idp_ga.m`](src/arm/solver_idp_ga.m)).

## Project Structure

```
config/      default parameters
robots/      robot definitions
src/         kinematics, arm, platform, env, planner, viz
tests/       unit tests
run_demo.m   command-line entry
mm_planner_gui.m
```

## License

[MIT](LICENSE)
