function [joints, T] = fk_chain(robot, q)
%FK_CHAIN Positions of all joint frames for a single configuration.
%   joints = FK_CHAIN(robot, q) returns an (n+1)-by-3 matrix with the
%   origin of the base frame followed by the origins of frames 1..n (mm).
%   [joints, T] = FK_CHAIN(...) also returns the 4-by-4 end transform.

n = size(robot.dh, 1);
joints = zeros(n + 1, 3);
T = eye(4);
for i = 1:n
    T = T * dh_matrix(robot, i, q(i));
    joints(i + 1, :) = T(1:3, 4).';
end
end
