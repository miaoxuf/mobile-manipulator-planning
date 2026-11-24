function tests = test_env
tests = functiontests(localfunctions);
end

function setupOnce(~)
run(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'setup_paths.m'));
end

function testBresenhamEndpointsAndSteps(tc)
rng(4);
for k = 1:50
    p0 = randi(20, 1, 2);  p1 = randi(20, 1, 2);
    c = bresenham_cells(p0, p1);
    verifyEqual(tc, c(1, :), p0);
    verifyEqual(tc, c(end, :), p1);
    if size(c, 1) > 1
        verifyLessThanOrEqual(tc, max(abs(diff(c)), [], 2), 1);   % 8-connected
    end
    verifyEqual(tc, size(c, 1), max(abs(p1 - p0)) + 1);
end
end

function testLineBlocked(tc)
occ = false(7, 7);
occ(4, 4) = true;
verifyTrue(tc, is_line_blocked(occ, [1 1], [7 7]));
verifyFalse(tc, is_line_blocked(occ, [1 1], [1 7]));
verifyFalse(tc, is_line_blocked(occ, [1 4], [4 4]));   % target cell ignored
end

function testMapFromAscii(tc)
occ = map_from_ascii({'.#.'; '..#'});
verifyEqual(tc, occ, logical([0 1 0; 0 0 1]));
verifyError(tc, @() map_from_ascii({'..'; '...'}), ?MException);
end

function testWarehouseMap(tc)
occ = warehouse_map();
verifyEqual(tc, size(occ), [20 20]);
verifyFalse(tc, occ(1, 1));
end

function testStationCandidates(tc)
occ = false(15, 15);
occ(8, 5:11) = true;
target = [5 8];
radius = [600 1500];
c = find_station_candidates(occ, target, radius, 300);
d = hypot(c.valid(:, 1) - target(1), c.valid(:, 2) - target(2)) * 300;
verifyTrue(tc, all(d >= radius(1) & d <= radius(2)));
verifyEqual(tc, d, c.dist, 'AbsTol', 1e-9);
verifyFalse(tc, any(occ(sub2ind(size(occ), c.valid(:, 1), c.valid(:, 2)))));
for k = 1:size(c.valid, 1)
    verifyFalse(tc, is_line_blocked(occ, c.valid(k, :), target));
end
verifyEqual(tc, size(c.ring, 1), size(c.valid, 1) + nnz(c.blocked));
end
