function c = viz_task_color(i)
%VIZ_TASK_COLOR Fixed categorical colour of task i.
%   Tasks beyond the eighth reuse the palette; use small multiples or a
%   secondary encoding (markers, labels) for that many tasks.

th = viz_theme();
c = th.categorical(mod(i - 1, size(th.categorical, 1)) + 1, :);
end
