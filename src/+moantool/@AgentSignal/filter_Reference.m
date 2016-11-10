function filter = filter_Reference(varargin)
% Allows the signal to track a certain reference profile

% Copyright is with the following author(s):
%
% (C) 2016 Michal Kvasnica, Slovak University of Technology in Bratislava
%          michal.kvasnica@stuba.sk
%
% This project is covered by the GNU GPL2 license. See COPYING for more
% information.

% set up the filter
filter = moantool.utils.FilterSetup;
filter.addField('value', 'parameter');

% the filter impacts the following calls:
filter.callback('set') = @on_set;
filter.callback('getVariables') = @on_variables;
filter.callback('instantiate') = @on_instantiate;
filter.callback('uninstantiate') = @on_uninstantiate;

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

out = obj.default_getVariables_handler('Reference', [obj.Dim(1), 1, obj.N]);

end

%------------------------------------------------
function out = on_instantiate(obj, varargin)
% called after the object was instantiated

out = obj.default_instantiate_handler('Reference', [obj.Dim(1), 1, obj.N]);

end

%------------------------------------------------
function out = on_uninstantiate(obj, varargin)
% called when the YALMIP representation of variables is removed

out = obj.default_uninstantiate_handler('Reference');

end

%------------------------------------------------
function obj = on_set(obj, value)
% called when the reference is to be changed


error(obj.check_set_value(value));
obj.Reference = value;

end
