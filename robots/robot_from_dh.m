function robot = robot_from_dh(dh, varargin)
%ROBOT_FROM_DH Build a robot struct from a D-H table.
%   robot = ROBOT_FROM_DH(dh) creates an n-joint revolute manipulator from
%   the n-by-3 [a alpha d] or n-by-4 [a alpha d offset] table `dh`
%   (a, d in mm; alpha, offset in deg).  The joint variable of joint i is
%   q(i) and the D-H angle is theta(i) = q(i) + offset(i).
%
%   Name-value options (defaults in brackets):
%     'Name'        robot name                          ['custom arm']
%     'Convention'  'standard' | 'modified' (Craig)     ['standard']
%     'QLim'        n-by-2 joint limits of q (deg)      [-180 180]
%     'QHome'       1-by-n home / start configuration   [zeros]
%     'Inertia'     joint moments of inertia (kg m^2)   [ones]
%     'OmegaAvg'    average joint speed (rad/s)         [ones]
%     'AlphaMax'    max joint acceleration (rad/s^2)    [Inf]
%     'BaseHeight'  arm base height above floor (mm)    [0]
%
%   Example:
%     dh = [0 90 89.2; -425 0 0; -392 0 0; 0 90 109.2; 0 -90 94.7; 0 0 82.3];
%     ur = robot_from_dh(dh, 'Name', 'UR5-like', 'BaseHeight', 400);

if size(dh, 2) == 3
    dh = [dh, zeros(size(dh, 1), 1)];
end
n = size(dh, 1);

p = inputParser;
p.FunctionName = 'robot_from_dh';
p.addParameter('Name', 'custom arm');
p.addParameter('Convention', 'standard');
p.addParameter('QLim', repmat([-180 180], n, 1));
p.addParameter('QHome', zeros(1, n));
p.addParameter('Inertia', ones(1, n));
p.addParameter('OmegaAvg', ones(1, n));
p.addParameter('AlphaMax', inf(1, n));
p.addParameter('BaseHeight', 0);
p.parse(varargin{:});
o = p.Results;

robot.name        = o.Name;
robot.convention  = o.Convention;
robot.dh          = dh;
robot.qlim        = o.QLim;
robot.q_home      = o.QHome;
robot.inertia     = o.Inertia;
robot.omega_avg   = o.OmegaAvg;
robot.alpha_max   = o.AlphaMax;
robot.base_height = o.BaseHeight;

robot = validate_robot(robot);
end
