function filter = filter_Max(varargin)
% Lower bound constraint

% Copyright is with the following author(s):
%
% (C) 2016 Michal Kvasnica, Slovak University of Technology in Bratislava
%          michal.kvasnica@stuba.sk
%
% This project is covered by the GNU GPL2 license. See COPYING for more
% information.

% set up the filter
filter = optiplan.utils.FilterSetup;
filter.addField('value', 'parameter');

% the filter impacts the following calls:
filter.callback('constraints') = @on_constraints;
filter.callback('set') = @on_set;
filter.callback('instantiate') = @on_instantiate;
filter.callback('uninstantiate') = @on_uninstantiate;
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

out = obj.default_getVariables_handler('Max', [obj.Dim(1), 1, obj.N]);

end

%------------------------------------------------
function out = on_instantiate(obj, varargin)
% called after the object was instantiated

out = obj.default_instantiate_handler('Max', [obj.Dim(1), 1, obj.N]);

end

%------------------------------------------------
function out = on_uninstantiate(obj, varargin)
% called when the YALMIP representation of variables is removed

out = obj.default_uninstantiate_handler('Max');

end

%------------------------------------------------
function out = on_constraints(obj, varargin)
% called when constructing constraints

out = [];
if isnumeric(obj.Max)
    % fixed bounds
    for i = 1:obj.Dim(1)
        % Do not include +/-Inf bounds
        if ~isinf(obj.Max(i))
            out = out + [ obj.Var(i, :, :) <= obj.Max(i) ];
        end
    end
else
    % parametric bounds
    out = out + [ obj.Var <= obj.Internal.parameters.Max ];
end

end

%------------------------------------------------
function obj = on_set(obj, value)
% called when the filter's values are changed

error(obj.check_set_value(value));
obj.Max = value;

end
