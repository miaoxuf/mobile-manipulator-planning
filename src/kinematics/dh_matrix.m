function A = dh_matrix(robot, i, q)
%DH_MATRIX Link transforms of joint i for a batch of joint angles.
%   A = DH_MATRIX(robot, i, q) returns a 4-by-4-by-N array with the
%   homogeneous transform from frame i-1 to frame i for the N joint
%   angles q (deg).
%
%   'standard' D-H : A = Rz(theta) Tz(d) Tx(a) Rx(alpha)
%   'modified' D-H : A = Rx(alpha) Tx(a) Rz(theta) Tz(d)   (Craig), where
%                    row i stores a_{i-1} and alpha_{i-1}.

a     = robot.dh(i, 1);
alpha = deg2rad(robot.dh(i, 2));
d     = robot.dh(i, 3);
theta = deg2rad(q(:) + robot.dh(i, 4));

N  = numel(theta);
ct = reshape(cos(theta), 1, 1, N);
st = reshape(sin(theta), 1, 1, N);
ca = cos(alpha);
sa = sin(alpha);

A = zeros(4, 4, N);
switch robot.convention
    case 'standard'
        A(1, 1, :) = ct;   A(1, 2, :) = -st * ca;  A(1, 3, :) =  st * sa;  A(1, 4, :) = a * ct;
        A(2, 1, :) = st;   A(2, 2, :) =  ct * ca;  A(2, 3, :) = -ct * sa;  A(2, 4, :) = a * st;
                           A(3, 2, :) =  sa;       A(3, 3, :) =  ca;       A(3, 4, :) = d;
    case 'modified'
        A(1, 1, :) = ct;       A(1, 2, :) = -st;                            A(1, 4, :) = a;
        A(2, 1, :) = st * ca;  A(2, 2, :) = ct * ca;  A(2, 3, :) = -sa;     A(2, 4, :) = -sa * d;
        A(3, 1, :) = st * sa;  A(3, 2, :) = ct * sa;  A(3, 3, :) =  ca;     A(3, 4, :) =  ca * d;
    otherwise
        error('dh_matrix:convention', 'Unknown D-H convention "%s".', robot.convention);
end
A(4, 4, :) = 1;
end
