function d = heading_delta(a, b)
%HEADING_DELTA Smallest absolute angle between two headings (deg).
%   d = HEADING_DELTA(a, b) returns a value in [0, 180].  NaN headings
%   (free / unspecified) give 0.

d = abs(mod(a - b + 180, 360) - 180);
d(isnan(d)) = 0;
end
