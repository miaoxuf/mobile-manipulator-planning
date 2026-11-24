# Mobile Manipulator Planning

[English](README.md) | 中文

面向栅格化仓库货物抓取的移动机械臂路径规划，兼顾能耗与抓取精度，基于 MATLAB 实现。本仓库为论文 *Path planning for mobile manipulator based on energy consumption and grasping accuracy constraints* 的优化代码。

## 环境

MATLAB R2020b 及以上。

## 快速开始

```matlab
setup_paths
out = run_demo;      % 规划示例任务，图片保存到 ./results
mm_planner_gui       % 图形界面
run_tests;           % 单元测试
```

## 使用

全部参数见 [`config/default_config.m`](config/default_config.m)。

```matlab
cfg = default_config();

% 机器人：D-H 表 [a alpha d offset]（mm, deg）及关节参数
cfg.robot = robot_from_dh(dh, 'QLim', qlim, 'QHome', q0, ...
                          'Inertia', I, 'OmegaAvg', w);

% 地图：'#' 为障碍物，一个字符对应一个栅格
cfg.env.map = map_from_ascii({'..........'; '.###..###.'; '..........'});
cfg.env.cell_size = 300;              % mm
cfg.env.start     = [1 1];
cfg.env.targets   = [3 9; 1 10];

cfg.arm.err_tol   = 10;               % 抓取误差容差 (mm)
out = run_demo(cfg);
```

机械臂求解器可通过 `cfg.arm.solver` 替换（参见 [`solver_idp_ga.m`](src/arm/solver_idp_ga.m)）。

## 目录结构

```
config/      默认参数
robots/      机器人定义
src/         运动学、机械臂、平台、环境、规划、绘图
tests/       单元测试
run_demo.m   命令行入口
mm_planner_gui.m
```

## 引用

如果本代码对你的研究有帮助，请引用我们的论文：

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

## 许可证

[MIT](LICENSE)
