function [tl, fig] = viz_canvas(parent, rows, cols, size_px, name)
%VIZ_CANVAS Create (or reuse) a container with a tiled layout.
%   [tl, fig] = VIZ_CANVAS(parent, rows, cols, size_px, name) returns a
%   tiledlayout inside `parent`.  If parent is empty a new figure of
%   size_px = [width height] pixels is created.  Any existing children of
%   a given parent (figure, panel or tab) are deleted first.

th = viz_theme();
if isempty(parent)
    fig = figure('Color', th.surface, 'Name', name, 'NumberTitle', 'off', ...
                 'Units', 'pixels');
    pos = fig.Position;
    fig.Position = [pos(1), max(pos(2) + pos(4) - size_px(2), 40), size_px];
    parent = fig;
else
    fig = ancestor(parent, 'figure');
    delete(parent.Children);
end
if isprop(parent, 'BackgroundColor')
    parent.BackgroundColor = th.surface;
end
tl = tiledlayout(parent, rows, cols, 'TileSpacing', 'compact', 'Padding', 'compact');
end
