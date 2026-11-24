function tests = test_planner
tests = functiontests(localfunctions);
end

function setupOnce(tc)
run(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'setup_paths.m'));
cfg = default_config();
cfg.verbose = false;
cfg.output.save = false;
cfg.env.targets = [12 3];
cfg.workspace.num_samples = 10000;
cfg.arm.ga.generations = 60;
tc.TestData.cfg = cfg;
tc.TestData.out = plan_grasp_tasks(cfg);
end

function testTaskSolved(tc)
out = tc.TestData.out;
t = out.tasks(1);
verifyTrue(tc, t.found);
verifyLessThanOrEqual(tc, t.best.error, tc.TestData.cfg.arm.err_tol);
verifyFalse(tc, out.map(t.best.station(1), t.best.station(2)));
verifyEqual(tc, t.best.path.path(end, :), t.best.station);
end

function testEnergyBookkeeping(tc)
b = tc.TestData.out.tasks(1).best;
verifyEqual(tc, b.E_platform, b.E_move + b.E_turn + b.E_rot, 'AbsTol', 1e-9);
verifyEqual(tc, b.E_total, b.E_platform + b.E_arm, 'AbsTol', 1e-9);
verifyEqual(tc, b.traj.energy, b.E_arm, 'RelTol', 1e-2);
end

function testSelectionIsOptimal(tc)
out = tc.TestData.out;
ev = out.tasks(1).evals;
w = out.cfg.select;
s = w.w_energy * ev.E_total + w.w_error * ev.error;
s(~ev.feasible) = inf;
verifyEqual(tc, min(s), w.w_energy * out.tasks(1).best.E_total + ...
                        w.w_error * out.tasks(1).best.error, 'AbsTol', 1e-9);
end

function testUserDefinedRingAndCustomRobot(tc)
cfg = tc.TestData.cfg;
cfg.workspace.radius = [900 1800];
cfg.robot.dh(3, 1) = 1100;                        % edit a link length
out = plan_grasp_tasks(cfg);
verifyEqual(tc, out.radius, [900 1800]);
verifyEmpty(tc, out.workspace);
verifyEqual(tc, height(out.summary), 1);
end

function testFiguresRender(tc)
out = tc.TestData.out;
makers = {@fig_workspace, @fig_station_candidates, @fig_joint_profiles, ...
          @fig_energy_breakdown, @fig_energy_summary, @fig_paths, ...
          @fig_arm_poses, @fig_convergence};
for k = 1:numel(makers)
    f = makers{k}(out);
    verifyTrue(tc, isgraphics(f));
    close(f);
end
end

function testBranchAndBoundIsExact(tc)
cfg = tc.TestData.cfg;
cfg.env.targets = [4 12; 14 13; 12 3];
cfg.arm.solver = @deterministic_solver;          % deterministic surrogate
cfg.select.prune = false;
full = plan_grasp_tasks(cfg);
cfg.select.prune = true;
fast = plan_grasp_tasks(cfg);
for i = 1:3
    verifyEqual(tc, fast.tasks(i).best.E_total, full.tasks(i).best.E_total, 'AbsTol', 1e-9);
    verifyEqual(tc, fast.tasks(i).best.error, full.tasks(i).best.error, 'AbsTol', 1e-9);
end
verifyLessThan(tc, sum(fast.summary.n_evaluated), sum(full.summary.n_evaluated));
end

function sol = deterministic_solver(robot, target, arm) %#ok<INUSD>
% Reaches every target; energy grows with the bearing change of joint 1.
sol.q = robot.q_home;
sol.err = 0.01 * hypot(target(1), target(2)) / 1000;
sol.energy = abs(atan2d(target(2), target(1))) / 10;
sol.cost = sol.energy + sol.err;
end
