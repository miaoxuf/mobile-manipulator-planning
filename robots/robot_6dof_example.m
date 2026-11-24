function robot = robot_6dof_example()
%ROBOT_6DOF_EXAMPLE Example 6-DOF industrial manipulator (standard D-H).
%   robot = ROBOT_6DOF_EXAMPLE() returns the robot struct used by the
%   default configuration.  Use it as a template for your own arm.

%            a (mm)  alpha (deg)  d (mm)  theta offset (deg)
dh = [          0        0         600        0
              300      -90           0        0
             1200        0           0        0
              300      -90        1200        0
                0      -90           0        0
                0      -90           0        0 ];

robot = robot_from_dh(dh, ...
    'Name',       '6-DOF example arm', ...
    'Convention', 'standard', ...
    'QLim',       [-160 160; -150 15; -200 80; -180 180; -120 120; -180 180], ...
    'QHome',      [0 -90 0 180 -90 0], ...
    'Inertia',    [0.5 0.3 0.4 0.6 0.2 0.4], ...     % kg m^2
    'OmegaAvg',   [2.0 1.5 1.0 2.5 3.0 2.0], ...     % rad/s
    'AlphaMax',   inf(1, 6), ...                     % rad/s^2
    'BaseHeight', 0);                                % mm
end
