function [P, T, Zj] = fk_batch(robot, Q)
%FK_BATCH Vectorised forward kinematics of the end-effector.
%   P = FK_BATCH(robot, Q) returns the N-by-3 end-effector positions (mm,
%   arm base frame) of the N-by-n joint configurations Q (deg).
%   [P, T] = FK_BATCH(...) also returns the 4-by-4-by-N end transforms.
%   [P, T, Zj] = FK_BATCH(...) also returns the N-by-n heights of the
%   frame origins 1..n (mm, base frame).
%
%   The rotation is propagated column by column (N-by-3 arrays) with
%   element-wise operations only, which is much faster than multiplying
%   4-by-4 matrices for the moderate batch sizes used by the optimisers.

[n, ~] = size(robot.dh);
N = size(Q, 1);
assert(size(Q, 2) == n, 'fk_batch: Q must have %d columns.', n);

a  = robot.dh(:, 1);
al = robot.dh(:, 2) * (pi / 180);
d  = robot.dh(:, 3);
th = (Q + robot.dh(:, 4).') * (pi / 180);
ca = cos(al);  sa = sin(al);
modified = strcmp(robot.convention, 'modified');

c1 = repmat([1 0 0], N, 1);         % columns of the rotation matrix
c2 = repmat([0 1 0], N, 1);
c3 = repmat([0 0 1], N, 1);
p  = zeros(N, 3);
wantZ = nargout > 2;
if wantZ
    Zj = zeros(N, n);
end

for i = 1:n
    ct = cos(th(:, i));
    st = sin(th(:, i));
    if modified
        % R * Rx(alpha) ; p += a * x ; R * Rz(theta) ; p += d * z
        p  = p + a(i) * c1;
        y  =  c2 * ca(i) + c3 * sa(i);
        c3 = -c2 * sa(i) + c3 * ca(i);
        x  =  c1 .* ct + y .* st;
        c2 = -c1 .* st + y .* ct;
        c1 = x;
        p  = p + d(i) * c3;
    else
        % R * Rz(theta) Tz(d) Tx(a) Rx(alpha)
        x  =  c1 .* ct + c2 .* st;
        y  = -c1 .* st + c2 .* ct;
        p  = p + a(i) * x + d(i) * c3;
        c1 = x;
        c2 =  y * ca(i) + c3 * sa(i);
        c3 = -y * sa(i) + c3 * ca(i);
    end
    if wantZ
        Zj(:, i) = p(:, 3);
    end
end
P = p;

if nargout > 1
    T = zeros(4, 4, N);
    T(1:3, 1, :) = reshape(c1.', 3, 1, N);
    T(1:3, 2, :) = reshape(c2.', 3, 1, N);
    T(1:3, 3, :) = reshape(c3.', 3, 1, N);
    T(1:3, 4, :) = reshape(p.', 3, 1, N);
    T(4, 4, :) = 1;
end
end
