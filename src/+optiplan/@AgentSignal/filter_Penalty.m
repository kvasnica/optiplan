function filter = filter_Penalty(varargin)
% Penalizes the signal in the cost function

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
filter.callback('objective') = @on_objective;
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

out = obj.default_getVariables_handler('Penalty', [obj.Dim(1), obj.Dim(1), obj.N]);

end

%------------------------------------------------
function out = on_instantiate(obj, varargin)
% called after the object was instantiated

out = obj.default_instantiate_handler('Penalty', [obj.Dim(1), obj.Dim(1), obj.N]);

end

%------------------------------------------------
function out = on_uninstantiate(obj, varargin)
% called when the YALMIP representation of variables is removed

out = obj.default_uninstantiate_handler('Penalty');

end

%------------------------------------------------
function out = on_objective(obj, varargin)
% called when constructing the cost function

out = 0;
if isempty(obj.Penalty) || (isnumeric(obj.Penalty) && all(all(obj.Penalty==0)))
    % do nothing if no penalty or zero penalty is used
	return
end

if obj.hasFilter('Reference')
    if isequal(obj.Reference, 'parameter')
        reference = obj.Internal.parameters.Reference;
    elseif size(obj.Reference, 3)~=obj.N
        assert(isequal(size(obj.Reference), obj.Dim), 'Reference must be a %dx%d vector.', obj.Dim(1), obj.Dim(2));
        reference = zeros(obj.Dim(1), obj.Dim(2), obj.N);
        for k = 1:obj.N
            reference(:, :, k) = obj.Reference;
        end
    else
        reference = obj.Reference;
    end
else
    reference = zeros(obj.Dim(1), 1, obj.N);
end

for k = 1:obj.N
    diff = obj.Var(:, :, k) - reference(:, :, k);
    if isequal(obj.Penalty, 'parameter')
        P = obj.Internal.parameters.Penalty(:, :, k);
    else
        P = obj.Penalty;
    end
    % TODO: support 1/Inf norms
    out = out + diff'*P*diff;
end

end

%------------------------------------------------
function obj = on_set(obj, P)
% called prior to property being set

% validate the penalty (empty penalty means no penalization)
if ~isempty(P)
    if isa(P, 'double')
        error(obj.validatePenalty(P));
    end
end
obj.Penalty = P;

end
