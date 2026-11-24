# Mobile Manipulator Planning

English | [中文](README_zh.md)

Energy- and accuracy-aware path planning for a mobile manipulator grasping goods in a grid warehouse, implemented in MATLAB. This repository contains the optimized code of our paper *Path planning for mobile manipulator based on energy consumption and grasping accuracy constraints*.

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

## Citation

If you find this code useful, please cite our paper:

```bibtex
@article{fang2025path,
  title   = {Path planning for mobile manipulator based on energy consumption and grasping accuracy constraints},
  author  = {Fang, Miaoxu and Xu, Zhen and Gai, Yizhou and Ye, Dabin and Liu, Tao and Tan, Ping and Tan, Kai},
  journal = {AIP Advances},
  volume  = {15},
  number  = {12},
  pages   = {125335},
  year    = {2025},
  doi     = {10.1063/5.0292185}
}
```

## License

[MIT](LICENSE)
