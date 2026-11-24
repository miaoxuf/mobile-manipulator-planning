function out = run_demo(cfg)
%RUN_DEMO Run the complete planning pipeline and produce all figures.
%   out = RUN_DEMO() uses DEFAULT_CONFIG.
%   out = RUN_DEMO(cfg) uses a user configuration, e.g.
%
%       setup_paths;
%       cfg = default_config();
%       cfg.env.targets = [5 10; 15 8];
%       cfg.arm.err_tol = 5;
%       out = run_demo(cfg);
%
%   Figures and a MAT file with the results are written to cfg.output.dir
%   when cfg.output.save is true.

setup_paths();
if nargin < 1 || isempty(cfg)
    cfg = default_config();
end

out = plan_grasp_tasks(cfg);
disp(out.summary);

figs = {
    '01_workspace',          @() fig_workspace(out)
    '02_station_candidates', @() fig_station_candidates(out)
    '03_joint_profiles',     @() fig_joint_profiles(out)
    '04_energy_breakdown',   @() fig_energy_breakdown(out)
    '05_energy_summary',     @() fig_energy_summary(out)
    '06_platform_paths',     @() fig_paths(out)
    '07_arm_poses',          @() fig_arm_poses(out)
    '08_convergence',        @() fig_convergence(out)
    };

for k = 1:size(figs, 1)
    fig = figs{k, 2}();
    drawnow;
    if cfg.output.save
        save_figure(fig, figs{k, 1}, cfg.output);
    end
end

if cfg.output.save
    save(fullfile(cfg.output.dir, 'plan_result.mat'), 'out');
    if cfg.verbose
        fprintf('Figures and results written to %s\n', cfg.output.dir);
    end
end
end
