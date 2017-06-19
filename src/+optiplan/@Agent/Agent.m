classdef Agent < optiplan.utils.OMPBaseClass
    % Class representing agents without obstacles

    % Copyright is with the following author(s):
    %
    % (C) 2016 Michal Kvasnica, Slovak University of Technology in Bratislava
    %          michal.kvasnica@stuba.sk
    %
    % This project is covered by the GNU GPL2 license. See COPYING for more
    % information.

    % Agent properties:
    %   .nx, .nu, .ny
    %   .N (prediction horizon)
    %   .BigBound
    %   .X, .U, .Y
    %     .Min
    %     .Max
    %     .Reference
    %     .Penalty
    %   .Size 
    %     .Value ([x_width; x_height])

    properties(SetAccess=protected)
        nx % Number of states
        nu % Number of inputs
        ny % Number of outputs
        N % prediction horizon
        X % vector of state predictions from x_0 to x_N
        U % vector of input predictions from u_0 to u_{N-1}
        Y % vector of output predictions from u_0 to u_{N-1}
        Size % size of the agent as [width; height]
    end
    properties
        ConstraintsFun % function of constraints cons(X, U, Y, agent)
        ObjectiveFun % objective function obj(X, U, Y, agent)
    end
    
    methods

        function obj = Agent(varargin)
            % Constructor
            %
            %   a = Agent('nx', nx, 'nu', nu, 'ny', ny, 'PredictionHorizon', N)

            if nargin==0
                return
            end
            
            % parse inputs
            ip = inputParser;
            ip.KeepUnmatched = false;
            ip.addParamValue('nx', [], @isnumeric);
            ip.addParamValue('nu', [], @isnumeric);
            ip.addParamValue('ny', [], @isnumeric);
            ip.addParamValue('PredictionHorizon', [], @isnumeric);
            ip.parse(varargin{:});
            S = ip.Results;

            % are all required inputs provided?
            if isempty(S.nx)
                error('The number of states must be provided.');
            end
            if isempty(S.nu)
                error('The number of inputs must be provided.');
            end
            if isempty(S.ny)
                error('The number of outputs must be provided.');
            end
            if isempty(S.PredictionHorizon)
                error('The prediction horizon must be provided.');
            end

            % initialize the object
            obj.nx = S.nx;
            obj.nu = S.nu;
            obj.ny = S.ny;
            obj.N = S.PredictionHorizon;

            % create and add signals
            obj.X = optiplan.AgentSignal([obj.nx 1], obj.N+1, 'X');
            obj.U = optiplan.AgentSignal([obj.nu 1], obj.N, 'U');
            obj.Y = optiplan.AgentSignal([obj.ny 1], obj.N, 'Y');

            % add geometry signals (currently only size)
            obj.Size = optiplan.AgentSignal([obj.ny 1], obj.N, 'Size');
            obj.Size.without(obj.Size.listFilters()); % remove all filters
            obj.Size.with('Value'); % add the "value" filter

            obj.Internal.instantiated = false;
        end
        
        function set.ConstraintsFun(obj, value)
            % constraints(X, U, Y, agent)
            
            assert(isa(value, 'function_handle'), 'The value must be a function handle.');
            if nargin(value)~=4
                error('The function must take 4 inputs: X, U, Y, agent');
            end
            obj.ConstraintsFun = value;
        end
        
        function set.ObjectiveFun(obj, value)
            % obj(X, U, Y, agent)
            
            assert(isa(value, 'function_handle'), 'The value must be a function handle.');
            if nargin(value)~=4
                error('The function must take 4 inputs: X, U, Y, agent');
            end
            obj.ObjectiveFun = value;
        end

%         function xn = update(obj, x, u)
%             % Returns the successor state
%             
%             error('Must be implemented in derived classes.');
%         end
% 
%         function y = output(obj, x, u)
%             % Returns the current output
%             
%             error('Must be implemented in derived classes.');
%         end

        function [info, Jopt] = optimize(obj, x0, sdpopts)
            % Optimizes the agent in absence of parameters

            narginchk(2, Inf);
            if nargin<3
                sdpopts = sdpsettings('verbose', 0);
            end

            % TODO: automatically reinstantiate if something has changed
            % (use wasModified, markAsModified(), markAsUnmodified)
            assert(obj.is_instantiated(), 'Agent is not instantiated.');

            params = obj.getParameters();
            assert(isempty(params), 'Only agents without parameters can be optimized here.');
            cons = obj.constraints();
            J = obj.objective();
            info = optimize(cons + [obj.X.Var(:, :, 1)==x0], J, sdpopts);
            Jopt = value(J);
        end

        function data = getOptimizerData(obj, prefix)
            % Returns info about parametric inputs and optimized variables
            
            if nargin<2
                prefix = '';
            end
            data = getOptimizerData@optiplan.utils.OMPBaseClass(obj, prefix);
            % add the initial condition to the front of the list
            data.inputs = [ {obj.X.Var(:, :, 1)}, data.inputs ];
            data.inputorder = [ {'x(0)'}, data.inputorder ];
            % requested variables
            data.outputs = { obj.U.Var(:, :, 1), ...
                squeeze(obj.X.Var), ...
                squeeze(obj.U.Var), ...
                squeeze(obj.Y.Var) };
            data.outputorder = {'u(0)', 'X', 'U', 'Y' };
        end
        
        function [opt, inputorder, outputorder] = getOptimizer(obj, solver, sdpopts)
            % Returns optimizer for this agent

            narginchk(2, Inf);
            assert(obj.is_instantiated(), 'Agent is not instantiated.');
            if nargin<3
                sdpopts = sdpsettings('verbose', 0);
            end
            options = sdpsettings(sdpopts, 'solver', solver);
            data = obj.getOptimizerData();
            cons = obj.constraints();
            J = obj.objective();
            opt = optimizer(cons, J, options, data.inputs, data.outputs);
            inputorder = data.inputorder;
            outputorder = data.outputorder;
        end
        
        function J = objective(obj)
            % Creates YALMIP objective for the object

            J = objective@optiplan.utils.OMPBaseClass(obj);
            
            % add a custom nonlinear objective
            if ~isempty(obj.ObjectiveFun)
                % aggregate variables over the whole horizon
                J = J + obj.ObjectiveFun(obj.X.squeeze(), ...
                    obj.U.squeeze(), obj.Y.squeeze(), obj);
            end
        end
        
        function cons = constraints(obj)
            % Creates YALMIP constraints of the object

            cons = constraints@optiplan.utils.OMPBaseClass(obj);
            
            % add custom nonlinear constraints over the whole horizon
            if ~isempty(obj.ConstraintsFun)
                cons = cons + obj.ConstraintsFun(obj.X.squeeze(), ...
                    obj.U.squeeze(), obj.Y.squeeze(), obj);
            end

            % adjust ymin/ymax constraints to geometry of the agent. simply
            % put, we need to tighten ymin/ymax bounds such that they are
            % not violated by (y_k+size_k/2) where size_k is the size of
            % the agent at the k-th prediction step. the size is a vector
            % of width (x-size) and height (y-size).
            for k = 1:length(obj.Y)
                if isequal(obj.Y.Max, 'parameter')
                    mx = obj.Y.Internal.parameters.Max(:, :, k);
                else
                    mx = obj.Y.Max;
                end
                if isequal(obj.Y.Min, 'parameter')
                    mi = obj.Y.Internal.parameters.Min(:, :, k);
                else
                    mi = obj.Y.Min;
                end
                s = obj.Size.value_or_var(k);
                if ~isequal(s, zeros(size(s)))
                    % don't forget that "y" is the center of the vehicle, so we
                    % have s/2 space to the left and s/2 to the right
                    cons = cons + [ mi+s/2 <= obj.Y.Var(:, :, k) <= mx-s/2 ];
                end
            end
        end
        
%         function display(obj)
% % TODO: enable the display() method
%             % display method
%
%             plural = @(s, n) [num2str(n) ' ' s repmat('s', 1, double(n~=1))];
%             if numel(obj)>1
%                 fprintf('Array of %d %ss\n', numel(obj), class(obj));
%             elseif isempty(obj.nx)
%                 fprintf('Empty %s\n', class(obj));
%             else
%                 disp(obj.display_internal());
%             end
%         end

    end
end
