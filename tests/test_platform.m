function tests = test_platform
tests = functiontests(localfunctions);
end

function setupOnce(tc)
run(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'setup_paths.m'));
tc.TestData.p = struct('k_move', 1, 'k_turn', 0.5, 'k_rot', 0.3);
end

function testHeadingDelta(tc)
verifyEqual(tc, heading_delta([0 0 0 270 -90 90], [90 180 270 0 270 NaN]), ...
            [90 180 90 90 0 0]);
end

function testStraightLineNoTurns(tc)
occ = false(5, 5);
r = energy_astar(occ, [1 1], 0, [5 1], 0, tc.TestData.p);
verifyTrue(tc, r.found);
verifyEqual(tc, r.length, 4);
verifyEqual(tc, [r.E_move r.E_turn r.E_rot], [4 0 0]);
end

function testFinalRotationAndInitialTurn(tc)
occ = false(5, 5);
p = tc.TestData.p;
r = energy_astar(occ, [1 1], 0, [1 4], 180, p);     % must head +y first
verifyEqual(tc, r.E_move, 3);
verifyEqual(tc, r.E_turn, 90 * p.k_turn);
verifyEqual(tc, r.E_rot, 90 * p.k_rot);
r = energy_astar(occ, [1 1], NaN, [1 4], NaN, p);   % free headings
verifyEqual(tc, r.E_total, 3);
end

function testStartEqualsGoal(tc)
r = energy_astar(false(3), [2 2], 0, [2 2], 270, tc.TestData.p);
verifyTrue(tc, r.found);
verifyEqual(tc, r.length, 0);
verifyEqual(tc, r.E_total, 90 * tc.TestData.p.k_rot);
end

function testUnreachable(tc)
occ = false(5, 5);
occ(3, :) = true;
r = energy_astar(occ, [1 1], 0, [5 5], NaN, tc.TestData.p);
verifyFalse(tc, r.found);
verifyEqual(tc, r.E_total, inf);
end

function testOptimalAgainstDijkstra(tc)
rng(11);
p = tc.TestData.p;
for trial = 1:25
    occ = rand(9, 8) < 0.25;
    free = find(~occ);
    if numel(free) < 2, continue, end
    k = free(randperm(numel(free), 2));
    [sx, sy] = ind2sub(size(occ), k(1));
    [gx, gy] = ind2sub(size(occ), k(2));
    h0 = 90 * randi([0 3]);
    hg = 90 * randi([0 3]);
    r = energy_astar(occ, [sx sy], h0, [gx gy], hg, p);
    ref = dijkstra_reference(occ, [sx sy], h0, [gx gy], hg, p);
    if isinf(ref)
        verifyFalse(tc, r.found);
    else
        verifyEqual(tc, r.E_total, ref, 'AbsTol', 1e-9);
        % the returned path is 4-connected and collision free
        verifyEqual(tc, sum(abs(diff(r.path)), 2), ones(size(r.path, 1) - 1, 1));
        verifyFalse(tc, any(occ(sub2ind(size(occ), r.path(:, 1), r.path(:, 2)))));
    end
end
end

function testMinHeapSorts(tc)
rng(8);
k = rand(500, 1);
h = MinHeap(4);
for i = 1:numel(k), h.push(k(i), i); end
out = zeros(numel(k), 1);
for i = 1:numel(k), [out(i), v] = h.pop(); verifyEqual(tc, k(v), out(i)); end
verifyEqual(tc, out, sort(k));
verifyTrue(tc, h.is_empty());
end

function testCostMapMatchesAstar(tc)
rng(12);
p = tc.TestData.p;
for trial = 1:15
    occ = rand(10, 9) < 0.25;
    free = find(~occ);
    k = free(randperm(numel(free), 1));
    [sx, sy] = ind2sub(size(occ), k);
    h0 = [NaN 0 90 180 270];
    h0 = h0(randi(5));
    cm = platform_cost_map(occ, [sx sy], h0, p);
    goals = free(randperm(numel(free), 6));
    for gi = goals.'
        [gx, gy] = ind2sub(size(occ), gi);
        hg = 90 * randi([0 3]);
        a = energy_astar(occ, [sx sy], h0, [gx gy], hg, p);
        b = extract_platform_path(cm, [gx gy], hg);
        verifyEqual(tc, b.found, a.found);
        if a.found
            verifyEqual(tc, b.E_total, a.E_total, 'AbsTol', 1e-9);
            verifyEqual(tc, platform_energy_bound(cm, [gx gy], hg), a.E_total, 'AbsTol', 1e-9);
            verifyEqual(tc, b.path(1, :), [sx sy]);
            verifyEqual(tc, b.path(end, :), [gx gy]);
        end
    end
end
end

function best = dijkstra_reference(occ, s, h0, g, hg, p)
% Plain Dijkstra on (x, y, heading) states without heuristic.
D = [1 0; 0 1; -1 0; 0 -1];
[R, C] = size(occ);
dist = inf(R, C, 4);
done = false(R, C, 4);
dist(s(1), s(2), mod(h0 / 90, 4) + 1) = 0;
while true
    tmp = dist;  tmp(done) = inf;
    [m, i] = min(tmp(:));
    if isinf(m), break, end
    [x, y, k] = ind2sub(size(dist), i);
    done(x, y, k) = true;
    for d = 1:4
        nx = x + D(d, 1);  ny = y + D(d, 2);
        if nx < 1 || ny < 1 || nx > R || ny > C || occ(nx, ny), continue, end
        c = m + p.k_move + p.k_turn * heading_delta(90 * (k - 1), 90 * (d - 1));
        dist(nx, ny, d) = min(dist(nx, ny, d), c);
    end
end
best = inf;
for k = 1:4
    best = min(best, dist(g(1), g(2), k) + p.k_rot * heading_delta(90 * (k - 1), hg));
end
end
