function [q, J, hist] = ga_search(cost_fn, seeds, lo, hi, opt)
%GA_SEARCH Bounded real-coded genetic algorithm with elitism.
%   [q, J, hist] = GA_SEARCH(cost_fn, seeds, lo, hi, opt) minimises cost_fn
%   over the box [lo, hi].  The rows of `seeds` (k-by-n, best first) are
%   inserted into the initial population, a quarter of the population is
%   sampled around the best seed and the rest uniformly in the box.  Each
%   generation applies tournament selection, blend (BLX-0.5) crossover,
%   annealed Gaussian mutation and keeps the opt.elite best individuals.
%
%   The run stops early when the best cost has not improved for
%   opt.stall_generations generations (0 disables early stopping).
%
%   cost_fn(Q) must accept an N-by-n matrix and return N-by-1 costs.
%   hist is the best cost of every generation.

stall = 0;
if isfield(opt, 'stall_generations'), stall = opt.stall_generations; end
n   = numel(lo);
NP  = opt.pop_size;
span = hi - lo;

seeds = seeds(1:min(size(seeds, 1), NP), :);
nNear = floor(NP / 4);
pop = [clip(seeds(1, :) + 0.05 * span .* randn(nNear, n), lo, hi);
       lo + span .* rand(NP - nNear, n)];
pop(1:size(seeds, 1), :) = seeds;
fit = cost_fn(pop);

hist = zeros(opt.generations, 1);
nChild = NP - opt.elite;
for g = 1:opt.generations
    [fit, idx] = sort(fit);
    pop = pop(idx, :);

    % tournament selection
    P1 = pop(tournament(fit, nChild, opt.tournament), :);
    P2 = pop(tournament(fit, nChild, opt.tournament), :);

    % BLX-0.5 crossover
    child = P1;
    doX = rand(nChild, 1) < opt.crossover_rate;
    w = -0.5 + 2 * rand(nChild, n);
    mixed = P1 + w .* (P2 - P1);
    child(doX, :) = mixed(doX, :);

    % annealed Gaussian mutation
    sigma = opt.mutation_scale * span * (1 - 0.9 * (g - 1) / opt.generations);
    mask = rand(nChild, n) < opt.mutation_rate;
    child = child + mask .* sigma .* randn(nChild, n);
    child = clip(child, lo, hi);

    pop = [pop(1:opt.elite, :); child];
    fit = [fit(1:opt.elite); cost_fn(child)];
    hist(g) = min(fit);

    % early stop when the best cost has stalled
    s = stall;
    if s > 0 && g > s && hist(g - s) - hist(g) <= 1e-6 * max(1, abs(hist(g)))
        hist = hist(1:g);
        break
    end
end

[J, k] = min(fit);
q = pop(k, :);
end

function idx = tournament(fit, m, k)
cand = randi(numel(fit), m, k);
[~, j] = min(fit(cand), [], 2);
idx = cand(sub2ind(size(cand), (1:m).', j));
end

function X = clip(X, lo, hi)
X = min(max(X, lo), hi);
end
