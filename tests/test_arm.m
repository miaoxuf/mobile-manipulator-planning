function tests = test_arm
tests = functiontests(localfunctions);
end

function setupOnce(tc)
run(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'setup_paths.m'));
tc.TestData.cfg = default_config();
tc.TestData.robot = robot_6dof_example();
end

function testQuinticBoundaryConditions(tc)
r = tc.TestData.robot;  arm = tc.TestData.cfg.arm;
qg = r.q_home + [40 -20 30 -60 15 90];
tr = arm_trajectory(r, qg, arm);
verifyEqual(tc, tr.q(1, :), r.q_home, 'AbsTol', 1e-9);
verifyEqual(tc, tr.q(end, :), qg, 'AbsTol', 1e-9);
verifyEqual(tc, tr.qd([1 end], :), zeros(2, 6), 'AbsTol', 1e-9);
verifyEqual(tc, tr.qdd([1 end], :), zeros(2, 6), 'AbsTol', 1e-9);
end

function testEnergyClosedFormMatchesIntegration(tc)
r = tc.TestData.robot;  arm = tc.TestData.cfg.arm;
arm.dt = 1e-4;
qg = r.q_home + [70 -35 20 -90 30 10];
tr = arm_trajectory(r, qg, arm);
[E, Ej] = arm_energy(r, qg, arm.T_nominal);
verifyEqual(tc, tr.energy_joint, Ej, 'RelTol', 1e-4);
verifyEqual(tc, tr.energy, E, 'RelTol', 1e-4);
verifyEqual(tc, arm_energy(r, r.q_home, arm.T_nominal), 0, 'AbsTol', 1e-12);
end

function testMotionTimeRespectsAverageSpeed(tc)
r = tc.TestData.robot;
dq = deg2rad([170 10 10 10 10 10]);
T = motion_time(r, dq, 0.1);
verifyLessThanOrEqual(tc, abs(dq) ./ T, r.omega_avg + 1e-12);
r.alpha_max(:) = 2;
T = motion_time(r, dq, 0.1);
verifyLessThanOrEqual(tc, (10 / sqrt(3)) * abs(dq) / T^2, r.alpha_max + 1e-9);
end

function testSolverReachesReachableTarget(tc)
r = tc.TestData.robot;  arm = tc.TestData.cfg.arm;
rng(3);
q = r.q_home + [60 10 -15 -40 0 0];
target = fk_batch(r, q);
sol = solver_idp_ga(r, target, arm);
verifyLessThan(tc, sol.err, arm.err_tol);
verifyEqual(tc, sol.floor_violation, 0);
verifyTrue(tc, all(sol.q >= r.qlim(:, 1).' & sol.q <= r.qlim(:, 2).'));
end

function testGaOnSphere(tc)
rng(5);
opt = tc.TestData.cfg.arm.ga;
f = @(X) sum((X - 3).^2, 2);
[q, J] = ga_search(f, zeros(1, 4), -10 * ones(1, 4), 10 * ones(1, 4), opt);
verifyLessThan(tc, J, 1e-2);
verifyEqual(tc, q, 3 * ones(1, 4), 'AbsTol', 0.1);
end

function testIdpOnSeparableFunction(tc)
opt = tc.TestData.cfg.arm.idp;
f = @(X) sum(abs(X - [5 -7 2]), 2);
[q, J] = idp_search(f, [0 0 0], -10 * ones(1, 3), 10 * ones(1, 3), opt);
verifyLessThan(tc, J, 1e-2);
verifyEqual(tc, q, [5 -7 2], 'AbsTol', 1e-2);
end

function testWorkspaceRing(tc)
rng(2);
ws = compute_workspace(tc.TestData.robot, 20000, [100 400]);
verifyGreaterThan(tc, ws.r_min, 0);
verifyGreaterThan(tc, ws.r_max, ws.r_min);
z = ws.points(ws.in_band, 3);
verifyTrue(tc, all(z >= 100 & z <= 400));
end

function testWarmStartSeedIsExactForRotatedTarget(tc)
r = tc.TestData.robot;
q = r.q_home + [20 10 -15 -40 0 0];
p0 = fk_batch(r, q);
phi = 35;
p1 = [cosd(phi) * p0(1) - sind(phi) * p0(2), sind(phi) * p0(1) + cosd(phi) * p0(2), p0(3)];
lib = struct('target', p0, 'q', q, 'ok', true);
seed = warm_start_seeds(lib, p1, r, 1, true);
verifyEqual(tc, fk_batch(r, seed), p1, 'AbsTol', 1e-6);
end

function testNullspaceRefineImproves(tc)
r = tc.TestData.robot;  arm = tc.TestData.cfg.arm;
q = r.q_home + [60 10 -15 -40 0 0];
target = fk_batch(r, q + [0 3 -2 4 0 0]);
J0 = arm_cost(r, q, target, arm);
[q1, J1] = nullspace_refine(r, q, target, arm, arm.nullspace);
verifyLessThan(tc, J1, J0);
[~, err] = arm_cost(r, q1, target, arm);
verifyLessThan(tc, err, 1);
verifyTrue(tc, all(q1 >= r.qlim(:, 1).' & q1 <= r.qlim(:, 2).'));
end
