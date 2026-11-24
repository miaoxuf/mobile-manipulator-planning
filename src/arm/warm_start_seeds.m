function seeds = warm_start_seeds(lib, target, robot, k, yaw_symmetric)
%WARM_START_SEEDS Initial configurations from previously solved targets.
%   seeds = WARM_START_SEEDS(lib, target, robot, k, yaw_symmetric) picks up
%   to k solutions from the library lib (struct with fields target m-by-3,
%   q m-by-n, ok m-by-1) that are closest to the new target (1-by-3, arm
%   base frame).  Feasible entries are preferred.
%
%   If the arm is yaw symmetric (HAS_BASE_YAW_SYMMETRY) the distance is
%   measured in (horizontal radius, height) and joint 1 of each seed is
%   rotated by the bearing difference, which maps a solution exactly onto
%   a target of the same radius and height.  Otherwise the Euclidean
%   distance of the targets is used and the seeds are taken unchanged.

n = size(robot.dh, 1);
seeds = zeros(0, n);
if isempty(lib.target) || k <= 0
    return
end

if yaw_symmetric
    rL = hypot(lib.target(:, 1), lib.target(:, 2));
    rT = hypot(target(1), target(2));
    dist = abs(rL - rT) + abs(lib.target(:, 3) - target(3));
else
    dist = sqrt(sum((lib.target - target(:).').^2, 2));
end
dist(~lib.ok) = dist(~lib.ok) + 1e9;            % prefer feasible solutions
[~, order] = sort(dist);
order = order(1:min(k, numel(order)));
seeds = lib.q(order, :);

if yaw_symmetric
    dphi = atan2d(target(2), target(1)) - ...
           atan2d(lib.target(order, 2), lib.target(order, 1));
    lo = robot.qlim(1, 1);  hi = robot.qlim(1, 2);
    for j = 1:numel(order)
        c = seeds(j, 1) + dphi(j) + 360 * (-2:2);     % equivalent angles
        inside = c >= lo & c <= hi;
        if any(inside)
            c = c(inside);
            [~, m] = min(abs(c - robot.q_home(1)));
            seeds(j, 1) = c(m);
        else
            seeds(j, 1) = min(max(c(3), lo), hi);
        end
    end
end
end
