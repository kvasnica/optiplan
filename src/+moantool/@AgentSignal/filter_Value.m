function filter = filter_Value(varargin)
% Fixed or parametric value
%
% use signal.value_or_var() to access the value

% Copyright is with the following author(s):
%
% (C) 2016 Michal Kvasnica, Slovak University of Technology in Bratislava
%          michal.kvasnica@stuba.sk
%
% This project is covered by the GNU GPL2 license. See COPYING for more
% information.

% set up the filter
filter = moantool.utils.FilterSetup;
filter.addField('Value', 'parameter');

% the filter impacts the following calls:
filter.callback('set') = @on_set;
filter.callback('getVariables') = @on_variables;

end

%------------------------------------------------
function out = on_variables(obj, varargin)
% called when filter's variables are requested
%
% Response: structure (or an array of structures) with following fields:
%
%  .Var: sdpvar representation of the introduced variable
%  .Dim: dimension of the variable
%  .Parametric: logical, if true, the variable will become part of the
%              vector of initial conditions

if isequal(obj.Value, 'parameter')
    % if it's a parameter, return the signal's variable
    out = struct('Var', obj.Var, ...
        'Dim', [obj.Dim, obj.N], ...
        'Parametric', true);
else
    out = [];
end

end

%------------------------------------------------
function obj = on_set(obj, value)
% called when the filter's values are changed

error(obj.check_set_value(value));
obj.Value = value;

end
