# Mobile Manipulator Planning under Energy and Grasping-Accuracy Constraints

[中文说明](README_zh.md)

MATLAB implementation of an iterative path-planning framework for a mobile
manipulator (a wheeled platform carrying a serial arm) that grasps goods in a
grid-mapped warehouse. Planning of the platform and the arm is decoupled:

1. **Workspace analysis** - Monte Carlo sampling of the arm's forward kinematics
   gives the horizontal working ring `[r_min, r_max]` at the height of the goods.
2. **Station selection** - every free grid cell inside the ring whose
   Bresenham line to the target is not obstructed is a candidate station.
3. **Platform planning** - a modified A\* / Dijkstra search on states
   *(x, y, heading)* gives the exact minimum platform energy (moving + turning +
   final in-place rotation) to every station and heading; one search serves all
   candidates.
4. **Arm optimisation** - the goal joint configuration of each station / heading
   is optimised by multi-start *iterative dynamic programming* (region
   contraction), a *genetic algorithm* and a *null-space gradient refinement*
   along the inverse-kinematics manifold. The objective weighs the motion energy
   of a rest-to-rest quintic joint trajectory against the end-effector position
   error, with penalties for exceeding the error tolerance or putting a joint
   below the floor clearance.
5. **Decision** - the station / heading with the lowest
   `w_energy * (E_platform + E_arm) + w_error * error` is selected. Combinations
   are optimised in order of increasing platform energy and the search stops as
   soon as the platform energy alone cannot beat the best score (exact branch and
   bound, since arm energy and error are non-negative).

Everything (robot, map, tasks, energy model, solver) is configurable; no data
files are needed.

## Requirements

* MATLAB R2020b or newer (developed and tested with R2025b)
* No toolbox is required. Optional: the Optimization / Global Optimization
  Toolbox if you want to plug in `fmincon`, `ga`, `particleswarm`, ... as solver.

## Quick start

```matlab
setup_paths            % add all folders to the path
out = run_demo;        % plan the example tasks, save figures to ./results
mm_planner_gui         % interactive GUI
run_tests;             % 35 unit tests
```

`run_demo` prints a summary table and writes eight figures (PNG + PDF):

| File | Content |
|---|---|
| `01_workspace` | Monte Carlo workspace, home pose and working ring |
| `02_station_candidates` | ring, blocked / unobstructed / feasible / selected stations per task |
| `03_joint_profiles` | joint angle, velocity and acceleration of the selected grasps |
| `04_energy_breakdown` | energy composition and grasp error of the best stations x headings |
| `05_energy_summary` | energy composition of the selected plan of every task |
| `06_platform_paths` | minimum-energy platform paths, stations and headings |
| `07_arm_poses` | 3-D home and grasp poses with the end-effector path |
| `08_convergence` | convergence of IDP, GA and local refinement |

## Performance

The default tasks (4 targets, 160 station x heading combinations) take well under
a second on a laptop. `examples/benchmark_planner.m` compares the options over
five random seeds (laptop, MATLAB R2025b):

| Mode | Time | Arm optimisations | Total energy (J) |
|---|---|---|---|
| exhaustive | 3.1 s | 160 | 525.3 +- 0.7 |
| exhaustive + warm start | 2.7 s | 160 | 525.9 +- 1.1 |
| **branch and bound (default)** | **0.24 s** | **14** | **525.3 +- 0.7** |
| branch and bound, no null-space refinement | 0.21 s | 14 | 528.8 +- 2.6 |

What makes it fast:

* **Vectorised kinematics** - forward kinematics propagates the rotation column by
  column with element-wise operations on whole batches of configurations.
* **Batched multi-start IDP** - all IDP starts are evaluated in one batch per
  joint sweep.
* **One platform search** - `platform_cost_map` runs a single heap-based Dijkstra
  search over *(x, y, heading)*; the exact energy to any station and heading is
  then a table lookup (`platform_energy_bound`, `extract_platform_path`).
* **Exact branch and bound** - the platform energy is a lower bound of the score,
  so most station / heading combinations never need an arm optimisation
  (`cfg.select.prune`). Set it to `false` to optimise every combination, e.g. for
  the full comparison in `04_energy_breakdown`.
* **Null-space refinement** - damped least-squares steps keep the end-effector on
  the target while the energy gradient is projected into the null space of the
  position Jacobian; this removes most of the run-to-run variance of the
  stochastic search.
* **Early stopping** of the GA (`cfg.arm.ga.stall_generations`) and optional
  **warm start** from earlier solutions (`cfg.arm.warm_start`), exploiting the
  yaw symmetry of joint 1 when present.

## Configuration

All parameters live in [`config/default_config.m`](config/default_config.m).
Modify the returned struct or copy the file:

```matlab
cfg = default_config();
cfg.env.targets  = [5 10; 15 8];      % grasp targets (grid cells)
cfg.env.start    = [1 1];
cfg.arm.err_tol  = 5;                 % mm
cfg.platform.k_turn = 0.2;            % J per degree of turning
out = run_demo(cfg);
```

### Robot

Any serial arm with revolute joints described by a D-H table can be used
(standard or modified/Craig convention, any number of joints):

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

See [`robots/robot_6dof_example.m`](robots/robot_6dof_example.m) for a complete
preset; new files named `robots/robot_*.m` automatically appear in the GUI.
In the GUI the D-H table, joint limits and dynamics can also be edited directly.

### Workspace

* `cfg.workspace.radius = []` estimates the working ring by Monte Carlo
  sampling of the current robot (`num_samples`, height band `z_band`).
* `cfg.workspace.radius = [r_min r_max]` sets it manually (mm).

### Map

The occupancy grid is a logical matrix (`true` = obstacle) addressed as
`map(x, y)`. Draw it as text:

```matlab
cfg.env.map = map_from_ascii({
    '..........'
    '.###..###.'
    '..........'});
cfg.env.cell_size = 250;   % mm
```

The GUI loads maps from `.mat` files (variable `map`) or ASCII `.txt` files.

### Energy model

| Term | Model |
|---|---|
| Arm | `sum_i  int |I_i * qdd_i * qd_i| dt` of a quintic rest-to-rest motion; duration `max(T_nominal, |dq_i| / omega_avg_i, sqrt(5.77 |dq_i| / alpha_max_i))` |
| Platform moving | `k_move` per grid step |
| Platform turning | `k_turn` per degree of heading change en route |
| Platform rotation | `k_rot` per degree of final in-place rotation at the station |

### Custom solver

`cfg.arm.solver` accepts any function `sol = f(robot, target, arm)` (or
`f(robot, target, arm, seeds)` to receive warm-start seeds) returning
`q`, `err`, `energy`, `cost` (optionally `floor_violation`, `history`).
Example that adds an `fmincon` refinement (Optimization Toolbox):

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

## Conventions

* Grid cell `(x, y)` addresses `map(x, y)`; figures draw `x` to the right and `y` upwards.
* World position of a cell centre: `(x, y) * cell_size` (mm).
* Platform heading is measured from `+x` towards `+y`; admissible headings are
  `cfg.platform.orientations` (default 0, 90, 180, 270 deg).
* The arm base frame is aligned with the platform heading; joint angles are in degrees.

## Project structure

```
config/        default_config.m             all parameters
robots/        robot presets, robot_from_dh, validate_robot
src/kinematics D-H transforms, batched forward kinematics
src/arm        workspace, quintic trajectory, energy, IDP, GA, null-space
               refinement, warm start, default solver
src/platform   heading-aware energy A*, Dijkstra cost map
src/env        maps, Bresenham line of sight, station candidates
src/planner    plan_grasp_tasks (complete pipeline)
src/utils      binary heap
src/viz        theme and figure functions
examples/      benchmark_planner.m
tests/         unit tests (matlab.unittest)
run_demo.m     command-line entry point
mm_planner_gui.m  GUI
```

## Citation

If this code helps your research, please cite our paper:

```
<citation to be added>
```

## License

[MIT](LICENSE)
