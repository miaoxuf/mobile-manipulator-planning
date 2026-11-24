function T = benchmark_planner(seeds)
%BENCHMARK_PLANNER Compare run time and solution quality of solver options.
%   T = BENCHMARK_PLANNER() plans the default tasks with
%     - exhaustive search vs. branch and bound (cfg.select.prune)
%     - with / without warm start (cfg.arm.warm_start)
%     - with / without null-space refinement (cfg.arm.nullspace.enable)
%   for several random seeds and returns a table with the mean run time,
%   number of arm optimisations and the mean / std of the total energy.
%   T = BENCHMARK_PLANNER(seeds) uses the given seeds (default 1:5).

if nargin < 1, seeds = 1:5; end
run(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'setup_paths.m'));

%        name                          prune  warm   nullspace
modes = {'exhaustive',                 false, false, true
         'exhaustive + warm start',    false, true,  true
         'branch and bound (default)', true,  false, true
         'B&B + warm start',           true,  true,  true
         'B&B, no null-space step',    true,  false, false};

nm = size(modes, 1);
time = zeros(nm, 1); evals = zeros(nm, 1); Em = zeros(nm, 1); Es = zeros(nm, 1);
for m = 1:nm
    t = zeros(numel(seeds), 1); e = t; ne = t;
    for k = 1:numel(seeds)
        cfg = default_config();
        cfg.verbose = false;
        cfg.output.save = false;
        cfg.seed = seeds(k);
        cfg.select.prune = modes{m, 2};
        cfg.arm.warm_start = modes{m, 3};
        cfg.arm.nullspace.enable = modes{m, 4};
        out = plan_grasp_tasks(cfg);
        t(k)  = out.runtime;
        e(k)  = sum(out.summary.E_total_J);
        ne(k) = sum(out.summary.n_evaluated);
    end
    time(m) = mean(t);  evals(m) = mean(ne);  Em(m) = mean(e);  Es(m) = std(e);
    fprintf('%-28s %6.2f s  %5.1f arm solves  E = %7.2f +- %.2f J\n', ...
            modes{m, 1}, time(m), evals(m), Em(m), Es(m));
end
T = table(modes(:, 1), time, evals, Em, Es, 'VariableNames', ...
          {'mode', 'time_s', 'arm_solves', 'E_total_mean_J', 'E_total_std_J'});
end
