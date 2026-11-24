function ax = viz_style_axes(ax, varargin)
%VIZ_STYLE_AXES Apply the common, recessive axes style.
%   VIZ_STYLE_AXES(ax) sets font, colours, ticks and a light grid.
%   VIZ_STYLE_AXES(ax, 'Grid', false) disables the grid.

p = inputParser;
p.addParameter('Grid', true);
p.parse(varargin{:});
th = viz_theme();

set(ax, 'FontName', th.font, 'FontSize', th.font_size, ...
        'Color', th.surface, 'XColor', th.ink2, 'YColor', th.ink2, 'ZColor', th.ink2, ...
        'GridColor', th.grid, 'GridAlpha', 1, 'MinorGridColor', th.grid, ...
        'LineWidth', th.thin, 'TickDir', 'out', 'TickLength', [0.008 0.008], ...
        'Box', 'off', 'Layer', 'bottom');
ax.Title.FontWeight = 'normal';
ax.Title.FontSize   = th.title_size;
ax.Title.Color      = th.ink;
if p.Results.Grid
    grid(ax, 'on');
else
    grid(ax, 'off');
end
hold(ax, 'on');
end
