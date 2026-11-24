function tests = test_kinematics
tests = functiontests(localfunctions);
end

function setupOnce(~)
run(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'setup_paths.m'));
end

function testHomePose(tc)
r = robot_6dof_example();
p = fk_batch(r, r.q_home);
verifyEqual(tc, p, [1200 -1200 600], 'AbsTol', 1e-9);
end

function testBatchMatchesChain(tc)
r = robot_6dof_example();
rng(1);
Q = r.qlim(:, 1).' + (r.qlim(:, 2) - r.qlim(:, 1)).' .* rand(20, 6);
P = fk_batch(r, Q);
for k = 1:size(Q, 1)
    J = fk_chain(r, Q(k, :));
    verifyEqual(tc, P(k, :), J(end, :), 'AbsTol', 1e-9);
end
end

function testFrameHeights(tc)
r = robot_6dof_example();
[~, ~, Zj] = fk_batch(r, r.q_home);
J = fk_chain(r, r.q_home);
verifyEqual(tc, Zj, J(2:end, 3).', 'AbsTol', 1e-9);
end

function testPlanarStandardVsModified(tc)
a1 = 400; a2 = 250; q = [30 -45];
std = robot_from_dh([a1 0 0; a2 0 0]);
mod = robot_from_dh([0 0 0; a1 0 0; a2 0 0], 'Convention', 'modified');
expected = [a1 * cosd(q(1)) + a2 * cosd(sum(q)), a1 * sind(q(1)) + a2 * sind(sum(q)), 0];
verifyEqual(tc, fk_batch(std, q), expected, 'AbsTol', 1e-9);
verifyEqual(tc, fk_batch(mod, [q 0]), expected, 'AbsTol', 1e-9);
end

function testOffsetColumn(tc)
r0 = robot_from_dh([300 0 0]);
r1 = robot_from_dh([300 0 0 90]);
verifyEqual(tc, fk_batch(r1, 0), fk_batch(r0, 90), 'AbsTol', 1e-9);
end

function testHomeOutsideLimitsRejected(tc)
verifyError(tc, @() robot_from_dh([100 0 0], 'QLim', [0 90], 'QHome', -10), ...
            'validate_robot:home');
end

function testYawSymmetryDetection(tc)
verifyTrue(tc, has_base_yaw_symmetry(robot_6dof_example()));
r = robot_from_dh([0 0 0; 300 90 0; 200 0 0], 'Convention', 'modified');
verifyTrue(tc, has_base_yaw_symmetry(r));
r = robot_from_dh([0 30 0; 300 90 0; 200 0 0], 'Convention', 'modified');
verifyFalse(tc, has_base_yaw_symmetry(r));
end
