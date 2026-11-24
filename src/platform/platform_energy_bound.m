function E = platform_energy_bound(cm, stations, headings)
%PLATFORM_ENERGY_BOUND Platform energy to many stations / headings at once.
%   E = PLATFORM_ENERGY_BOUND(cm, stations, headings) returns the exact
%   minimum platform energy (moving + turning + final rotation) for each
%   row of `stations` (K-by-2 cells) and the matching entry of `headings`
%   (K-by-1 deg).  Since arm energy and grasp error are non-negative, it is
%   also a lower bound of the total score used for branch and bound.

R = cm.size(1);  C = cm.size(2);
K = size(stations, 1);
idx = stations(:, 1) + (stations(:, 2) - 1) * R;
G = reshape(cm.g, R * C, 4);
G = G(idx, :);                                          % K-by-4
rot = cm.p.k_rot * heading_delta(90 * (0:3), headings(:));   % K-by-4
E = min(G + rot, [], 2);
E = reshape(E, K, 1);
end
