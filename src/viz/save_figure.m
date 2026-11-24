function files = save_figure(fig, name, out_cfg)
%SAVE_FIGURE Export a figure in all configured formats.
%   files = SAVE_FIGURE(fig, name, out_cfg) writes <out_cfg.dir>/<name>.<ext>
%   for every extension in out_cfg.formats ('png', 'pdf', 'svg', 'jpg',
%   'tif', 'eps').  Raster files use out_cfg.dpi.

%   Figures that set fig.UserData.content = 'image' (dense point clouds,
%   transparent 3-D scenes) are embedded as high-resolution images in
%   PDF/EPS instead of vector graphics.

if ~exist(out_cfg.dir, 'dir')
    mkdir(out_cfg.dir);
end
content = 'vector';
if isstruct(fig.UserData) && isfield(fig.UserData, 'content')
    content = fig.UserData.content;
end
% hide interactive axes toolbars so they never end up in the export
ax = findall(fig, 'Type', 'axes');
for k = 1:numel(ax)
    if isprop(ax(k), 'Toolbar') && ~isempty(ax(k).Toolbar)
        ax(k).Toolbar.Visible = 'off';
    end
end
drawnow;

pad = {};
if ~isMATLABReleaseOlderThan('R2023b')
    pad = {'Padding', 12};                         % keep titles off the edge
end
files = {};
for k = 1:numel(out_cfg.formats)
    ext = lower(out_cfg.formats{k});
    f = fullfile(out_cfg.dir, [name '.' ext]);
    switch ext
        case 'svg'
            print(fig, f, '-dsvg', '-vector');
        case {'pdf', 'eps'}
            if strcmp(content, 'image')
                exportgraphics(fig, f, 'ContentType', 'image', ...
                               'Resolution', out_cfg.dpi, 'BackgroundColor', fig.Color, pad{:});
            else
                exportgraphics(fig, f, 'ContentType', 'vector', ...
                               'BackgroundColor', fig.Color, pad{:});
            end
        otherwise
            exportgraphics(fig, f, 'Resolution', out_cfg.dpi, ...
                           'BackgroundColor', fig.Color, pad{:});
    end
    files{end + 1} = f; %#ok<AGROW>
end
end
