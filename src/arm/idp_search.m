function [q, J, hist] = idp_search(cost_fn, q, lo, hi, opt, range)
%IDP_SEARCH Iterative dynamic programming with region contraction.
%   [q, J, hist] = IDP_SEARCH(cost_fn, q0, lo, hi, opt) improves the joint
%   vector(s) q0 by repeated sweeps over the joints.  For joint i a grid
%   of opt.grid candidate angles is laid over the current search region
%   around q(i) (clipped to [lo(i), hi(i)]) and the best candidate is kept.
%   After each sweep the region contracts by opt.shrink, as in Luus' IDP.
%
%   q0 may hold several starting points (k-by-n).  They are searched
%   simultaneously - every cost evaluation is one batch over all starts -
%   and are returned as k-by-n with costs J (k-by-1), best first.
%
%   [...] = IDP_SEARCH(..., range) starts from the given region widths
%   (deg) instead of the full joint range.
%
%   cost_fn(Q) must accept an N-by-n matrix and return N-by-1 costs.
%   hist is the best cost (over all starts) after every sweep.

[k, n] = size(q);
if nargin < 6 || isempty(range)
    range = 2 * (hi - lo);          % first sweep covers the full range
end
G = opt.grid;
u = linspace(0, 1, G);
J = cost_fn(q);
hist = zeros(opt.max_iter, 1);
rows = (1:k).';

for it = 1:opt.max_iter
    for i = 1:n
        a = max(lo(i), q(:, i) - range(i) / 2);           % k-by-1
        b = min(hi(i), q(:, i) + range(i) / 2);
        cand = [a + (b - a) .* u, q(:, i)];               % k-by-(G+1)
        Q = repelem(q, G + 1, 1);
        Q(:, i) = reshape(cand.', [], 1);
        Jc = reshape(cost_fn(Q), G + 1, k);
        [Jmin, idx] = min(Jc, [], 1);
        better = Jmin(:) < J;
        if any(better)
            sel = (rows - 1) * (G + 1) + idx(:);
            q(better, i) = Q(sel(better), i);
            J(better) = Jmin(better);
        end
    end
    hist(it) = min(J);
    range = range * opt.shrink;
    if max(range) < opt.min_range
        hist = hist(1:it);
        break
    end
end

[J, order] = sort(J);
q = q(order, :);
end
