function root = setup_paths()
%SETUP_PATHS Add all project folders to the MATLAB search path.
%   root = SETUP_PATHS() returns the project root folder.

root = fileparts(mfilename('fullpath'));
addpath(root, ...
        fullfile(root, 'config'), ...
        fullfile(root, 'robots'), ...
        genpath(fullfile(root, 'src')));
end
