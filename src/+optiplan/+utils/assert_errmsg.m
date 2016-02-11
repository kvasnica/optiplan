function assert_errmsg(msg, varargin)
% raises an error is MSG does not contain given string(s)

error(nargchk(2, Inf, nargin));

for i = 1:length(varargin)
    if ~isempty(strfind(msg, varargin{i}))
        return
    end
end
assert(~isempty(strfind(msg, varargin{1})));
