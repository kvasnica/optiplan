function h = plotRectangle(varargin)

ip = inputParser;
ip.addParamValue('Position', []);
ip.addParamValue('Size', []);
ip.addParamValue('MinSize', []);
ip.addParamValue('Color', 'y');
ip.addParamValue('Wire', false);
ip.addParamValue('EdgeColor', 'k');
ip.parse(varargin{:});
Options = ip.Results;

assert(length(Options.Position)==2, 'Only 2D rectangles can be plotted for now.');
assert(length(Options.Size)==2, 'Only 2D rectangles can be plotted for now.');

if any(Options.Size==0) && ~isempty(Options.MinSize)
    Options.Size = max(Options.Size, Options.MinSize);
end
    

pos = [Options.Position(1)-Options.Size(1)/2, ...
    Options.Position(2)-Options.Size(2)/2, ...
    Options.Size(1), Options.Size(2)];
h = rectangle('Position', pos);
if Options.Wire
    set(h, 'EdgeColor', Options.Color);
else
    set(h, 'FaceColor', Options.Color);
    set(h, 'EdgeColor', Options.EdgeColor);
end

if nargout==0
    clear h
end
end
