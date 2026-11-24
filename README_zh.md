# 考虑能耗与抓取精度约束的移动机械臂路径规划

[English](README.md)

本仓库是移动机械臂（轮式移动平台 + 串联机械臂）在栅格化仓库中抓取货物的迭代式路径规划框架的 MATLAB 实现。移动平台与机械臂的规划相互解耦：

1. **工作空间分析**：对机械臂正运动学做蒙特卡洛采样，得到货物高度附近的水平工作环 `[r_min, r_max]`。
2. **站位点筛选**：工作环内的空闲栅格中，到目标点的 Bresenham 直线不被障碍物遮挡的，作为候选站位点。
3. **平台路径规划**：在 *(x, y, 朝向)* 状态空间上运行改进 A\* / Dijkstra 搜索，得到到达每个站位点、每个朝向的精确最小平台能耗（移动、转向和到站后的原地旋转）。一次搜索即可覆盖所有候选。
4. **机械臂优化**：对每个“站位点 × 朝向”组合，依次用多起点**迭代动态规划**（搜索区域收缩）、**遗传算法**和沿逆解流形的**零空间梯度精修**优化目标关节角。目标函数在两项之间加权：一是五次多项式静止到静止轨迹的运动能耗，二是末端定位误差。误差超出容差、关节低于离地间隙时会施加惩罚。
5. **决策**：选取 `w_energy * (E_平台 + E_机械臂) + w_error * 误差` 最小的站位点与朝向。各组合按平台能耗从小到大依次求解，一旦平台能耗本身已不可能优于当前最优，就停止搜索。由于机械臂能耗和误差都非负，这一分支定界是精确的。

机器人、地图、任务、能耗模型、求解器全部可配置，运行时不依赖任何数据文件。

## 环境要求

* MATLAB R2020b 及以上（开发与测试版本为 R2025b）
* 不依赖任何工具箱。如需接入 `fmincon`、`ga`、`particleswarm` 等求解器，可选装 Optimization / Global Optimization Toolbox。

## 快速开始

```matlab
setup_paths            % 添加路径
out = run_demo;        % 规划示例任务，图片保存到 ./results
mm_planner_gui         % 图形界面
run_tests;             % 35 个单元测试
```

`run_demo` 会打印结果汇总表，并输出 8 张图（PNG + PDF）：

| 文件 | 内容 |
|---|---|
| `01_workspace` | 蒙特卡洛工作空间、零位姿态与工作环 |
| `02_station_candidates` | 各任务的工作环，以及被遮挡 / 无遮挡 / 可行 / 最终选定的站位点 |
| `03_joint_profiles` | 选定方案的关节角、角速度、角加速度曲线 |
| `04_energy_breakdown` | 最优若干站位点在各朝向下的能耗构成与抓取误差 |
| `05_energy_summary` | 各任务最终方案的能耗构成 |
| `06_platform_paths` | 平台最小能耗路径、站位点与朝向 |
| `07_arm_poses` | 三维零位 / 抓取姿态及末端轨迹 |
| `08_convergence` | IDP、GA、局部精修的收敛曲线 |

## 性能

默认的 4 个任务（共 160 个“站位点 × 朝向”组合）在笔记本上不到 1 秒即可完成。`examples/benchmark_planner.m` 会在 5 个随机种子下对比各选项（笔记本电脑，MATLAB R2025b）：

| 模式 | 耗时 | 机械臂优化次数 | 总能耗 (J) |
|---|---|---|---|
| 穷举 | 3.1 s | 160 | 525.3 ± 0.7 |
| 穷举 + 热启动 | 2.7 s | 160 | 525.9 ± 1.1 |
| **分支定界（默认）** | **0.24 s** | **14** | **525.3 ± 0.7** |
| 分支定界，不做零空间精修 | 0.21 s | 14 | 528.8 ± 2.6 |

提速的关键在于：

* **向量化运动学**：正运动学按列递推旋转矩阵，整批构型只用逐元素运算。
* **批量多起点 IDP**：每次关节扫描时，所有起点合并为一批同时评估。
* **一次平台搜索**：`platform_cost_map` 用基于二叉堆的 Dijkstra 在 *(x, y, 朝向)* 上只搜索一次，之后到任意站位点、任意朝向的精确能耗都只是查表（`platform_energy_bound`、`extract_platform_path`）。
* **精确分支定界**：平台能耗是评分的下界，因此绝大多数组合根本不需要做机械臂优化（`cfg.select.prune`）。设为 `false` 会优化全部组合，例如需要 `04_energy_breakdown` 完整对比图时。
* **零空间精修**：用阻尼最小二乘把末端保持在目标点上，同时把能耗梯度投影到位置雅可比的零空间里下降。这一步消除了随机搜索在不同运行之间的大部分波动。
* GA 的**停滞早停**（`cfg.arm.ga.stall_generations`），以及可选的**热启动**（`cfg.arm.warm_start`）：复用已有解作为初值；如果关节 1 具有偏航对称性，还会把已有解的 q1 平移到新目标的方位上。

## 参数配置

全部参数集中在 [`config/default_config.m`](config/default_config.m)，可以修改返回的结构体，也可以复制该文件另建一份：

```matlab
cfg = default_config();
cfg.env.targets  = [5 10; 15 8];      % 目标点（栅格坐标）
cfg.env.start    = [1 1];
cfg.arm.err_tol  = 5;                 % mm
cfg.platform.k_turn = 0.2;            % 每度转向能耗 (J)
out = run_demo(cfg);
```

### 机器人

支持任意由 D-H 表描述的旋转关节串联机械臂：标准或改进（Craig）D-H，关节数不限。

```matlab
%      a(mm) alpha(deg) d(mm) offset(deg)
dh = [   0     90       89.2    0
      -425      0        0      0
      -392      0        0      0
         0     90      109.2    0
         0    -90       94.7    0
         0      0       82.3    0 ];
cfg.robot = robot_from_dh(dh, 'Name', 'my arm', ...
    'QLim',  repmat([-180 180], 6, 1), 'QHome', [0 -90 0 -90 0 0], ...
    'Inertia', [0.5 0.5 0.3 0.1 0.1 0.05], 'OmegaAvg', ones(1, 6), ...
    'AlphaMax', 5 * ones(1, 6), 'BaseHeight', 400);
```

完整预设见 [`robots/robot_6dof_example.m`](robots/robot_6dof_example.m)。放在 `robots/` 下、以 `robot_*.m` 命名的文件会自动出现在 GUI 的下拉框中；也可以直接在 GUI 里编辑 D-H 表、关节范围和动力学参数。

### 工作空间

* `cfg.workspace.radius = []`：按当前机器人做蒙特卡洛采样，自动估计工作环（参数为 `num_samples` 和高度带 `z_band`）。
* `cfg.workspace.radius = [r_min r_max]`：手动指定工作环（mm）。

### 地图

占据栅格是逻辑矩阵（`true` 表示障碍物），按 `map(x, y)` 索引。可以用文本绘制：

```matlab
cfg.env.map = map_from_ascii({
    '..........'
    '.###..###.'
    '..........'});
cfg.env.cell_size = 250;   % mm
```

GUI 可以加载 `.mat`（变量名 `map`）或 ASCII `.txt` 地图。

### 能耗模型

| 项 | 模型 |
|---|---|
| 机械臂 | 五次多项式静止到静止运动的 `sum_i ∫|I_i·q̈_i·q̇_i| dt`；运动时长取 `max(T_nominal, |Δq_i|/ω_avg_i, sqrt(5.77|Δq_i|/α_max_i))` |
| 平台移动 | 每移动一格 `k_move` |
| 平台转向 | 行进中每转 1° 耗 `k_turn` |
| 平台旋转 | 到站后原地旋转每 1° 耗 `k_rot` |

### 自定义求解器

`cfg.arm.solver` 可以设为任意函数 `sol = f(robot, target, arm)`（如需接收热启动初值，可写成 `f(robot, target, arm, seeds)`），返回 `q`、`err`、`energy`、`cost` 字段（可选 `floor_violation`、`history`）。下面的例子在默认求解器之后再加一步 `fmincon` 精修（需要 Optimization Toolbox）：

```matlab
function sol = solver_fmincon(robot, target, arm)
sol = solver_idp_ga(robot, target, arm);
lo = robot.qlim(:, 1).';  hi = robot.qlim(:, 2).';
f = @(q) arm_cost(robot, q, target, arm);
q = fmincon(f, sol.q, [], [], [], [], lo, hi, [], optimoptions('fmincon', 'Display', 'off'));
[J, err, E, v] = arm_cost(robot, q, target, arm);
if J < sol.cost
    sol.q = q;  sol.err = err;  sol.energy = E;  sol.cost = J;  sol.floor_violation = v;
end
end
```

```matlab
cfg.arm.solver = @solver_fmincon;
```

## 约定

* 栅格 `(x, y)` 对应 `map(x, y)`；图中 `x` 轴向右、`y` 轴向上。
* 栅格中心的世界坐标为 `(x, y) * cell_size`（mm）。
* 平台朝向从 `+x` 轴量向 `+y` 轴；可选朝向由 `cfg.platform.orientations` 指定，默认为 0、90、180、270°。
* 机械臂基座坐标系与平台朝向一致；关节角单位为度。

## 目录结构

```
config/        default_config.m             全部参数
robots/        机器人预设、robot_from_dh、validate_robot
src/kinematics D-H 变换、批量正运动学
src/arm        工作空间、五次多项式轨迹、能耗、IDP、GA、零空间精修、热启动、默认求解器
src/platform   考虑朝向的能耗 A*、Dijkstra 代价图
src/env        地图、Bresenham 视线判断、候选站位点
src/planner    plan_grasp_tasks（完整流程）
src/utils      二叉堆
src/viz        绘图主题与各类图
examples/      benchmark_planner.m
tests/         单元测试（matlab.unittest）
run_demo.m     命令行入口
mm_planner_gui.m  图形界面
```

## 引用

如果本代码对你的研究有帮助，请引用我们的论文：

```
<引用信息待补充>
```

## 许可证

[MIT](LICENSE)
