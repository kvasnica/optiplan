classdef Planner < optiplan.utils.OMPBaseClass
    % Class representing MPC-based path planners
    %
    %   p = Planner(agent, obstacles, 'solver', ...)

    % Copyright is with the following author(s):
    %
    % (C) 2016 Michal Kvasnica, Slovak University of Technology in Bratislava
    %          michal.kvasnica@stuba.sk
    %
    % This project is covered by the GNU GPL2 license. See COPYING for more
    % information.

    properties(SetAccess=protected, Hidden)
        Agent
        Obstacles
        Optimizer
        BigBound      % default big-M bound
        MinSeparation % minimal separation between the agent and the obstacle
    end
    properties
        Parameters
    end
    
    methods
        
        function obj = Planner(agent, obstacles, varargin)
            % Constructor for Planner objects
            %
            %   p = Planner(agent, obstacles, 'solver', ...)
            
            if nargin == 0
                return
            end
            narginchk(2, Inf);
            assert(isa(agent, 'optiplan.Agent'), 'The first input must be an Agent object.');
			% parse inputs	
            ip = inputParser;
            ip.KeepUnmatched = false;
            ip.addParamValue('solver', 'gurobi', @ischar);
            ip.addParamValue('MinSeparation', zeros(agent.ny, 1));
            ip.addParamValue('BigBound', 1e4);
            ip.parse(varargin{:});
            options = ip.Results;

            % copy agent/obstacles and instantiate them
            obj.Agent = copy(agent);
            obj.Agent.instantiate();
            if ~isempty(obstacles)
                assert(isa(obstacles, 'optiplan.Obstacle'), 'The second input must be an Obstacle object.');
                obj.Obstacles = copy(obstacles);
                for i = 1:length(obj.Obstacles)
                    obj.Obstacles(i).instantiate();
                end
            end
            obj.Internal.instantiated = true;
            
            % minimal separation gap between the agent and the obstacles
            obj.MinSeparation = options.MinSeparation;
            % big-M bound
            obj.BigBound = options.BigBound;
            
            % which variables are parametric?
            parameters = obj.getParameters();
            names = parameters.keys();
            values = parameters.values();
            % information about parameters
            param_info = cell(1, length(names));
            for i = 1:length(parameters)
                param_info{i} = obj.signal_info(names{i}, values{i});
            end
            % which modules contain a parametric variable?
            modules = { 'Agent', 'Obstacles' };
            % create module placeholders in Parameters
            for i = 1:length(modules)
                obj.Parameters.(modules{i}) = [];
            end
            % create placeholder for each parameter
            for i = 1:length(param_info)
                pi = param_info{i};
                obj.Parameters.(pi.module)(pi.module_index).(pi.signal).(pi.property) = NaN(pi.dim(1), pi.dim(2)*pi.dim(3));
            end

            [opt, data] = obj.getOptimizer(options.solver);
            obj.Optimizer.opt = opt;
            obj.Optimizer.inputs = data.inputorder;
            obj.Optimizer.parameters = data.inputorder(2:end); % kick out x(1)
            obj.Optimizer.outputs = data.outputorder;
            obj.Optimizer.param_info = param_info;
            obj.Optimizer.agent_params = parameters;
        end

        function [opt, data] = getOptimizer(obj, solver)
            % returns optimizer for this planner

            constraints = obj.constraints();
            objective = obj.objective();
            data = obj.Agent.getOptimizerData('Agent.');
            merge_fields = fieldnames(data);
            for i = 1:length(obj.Obstacles)
                odata = obj.Obstacles(i).getOptimizerData(sprintf('Obstacles(%d).', i));
                % merge odata into data
                for j = 1:length(merge_fields)
                    f = merge_fields{j};
                    data.(f) = [ data.(f), odata.(f)];
                end
            end
            options = sdpsettings('verbose', 0, 'solver', solver);
            opt = optimizer(constraints, objective, options, data.inputs, data.outputs);
        end
        
        function cons = constraints(obj)
            % Creates YALMIP constraints of the object
            
            cons = obj.Agent.constraints();
            if ~isempty(obj.Obstacles)
                cons = cons + obj.Obstacles.constraints(obj.Agent, obj.MinSeparation, obj.BigBound);
            end
        end
        
        function J = objective(obj)
            % Returns the optimization objective as a YALMIP variable
            
            J = obj.Agent.objective();
            if ~isempty(obj.Obstacles)
                J = J + obj.Obstacles.objective();
            end
        end

        function listParameters(obj)
            % Returns/displays parameters of the object
            
            if ~isempty(inputname(1))
                prefix = [inputname(1) '.Parameters.'];
            else
                prefix = 'obj.Parameters.';
            end
            listParameters@optiplan.utils.OMPBaseClass(obj, prefix);
        end
        
        function [inputs, missing] = readParameters(obj)
            % Prepares inputs for the optimizer
            
            % this function needs to be as efficient as possible
            names = obj.Optimizer.parameters;
            infos = obj.Optimizer.param_info;
            n = length(names);
            inputs = cell(1, n);
            missing = false(1, n);
            for i = 1:n
                pi = infos{i};
                value = obj.Parameters.(pi.module)(pi.module_index).(pi.signal).(pi.property);
                if isequal(value, 'parameter') || isempty(value) || all(all(isnan(value)))
                    value = [];
                    missing(i) = true;
                elseif size(value, 2)==pi.dim(2)
                    % automatically expand the value to horizon-N
                    value = repmat(value, 1, pi.dim(3));
                elseif size(value, 2)~=pi.dim(2)*pi.dim(3)
                    error('Parameter.%s(%d).%s.%s must be either a %dx%d or a %dx%d matrix.', ...
                        pi.module, pi.module_index, pi.signal, pi.property, ...
                        pi.dim(1), pi.dim(2), ...
                        pi.dim(1), pi.dim(2)*pi.dim(3));
                end
                inputs{i} = value;
            end
            % missing = names(missing);
        end
        
        function k = listMissing(obj, prefix)
            % Returns list of parameters that have not been set
            
            if nargin<2
                prefix = inputname(1);
            end
            [~, missing] = obj.readParameters();
            % recreate the parameter map
            k = obj.Optimizer.agent_params.keys;
            v = obj.Optimizer.agent_params.values;
            % kick out those which are not missing
            k = k(missing);
            v = v(missing);
            if isempty(k)
                % nothing is missing
                if nargout == 0, clear k, end
                return
            end
            if nargout==0
                prefix = [prefix '.Parameters.'];
                obj.Agent.listParameters(prefix, containers.Map(k, v));
                clear k
            end
        end
        
        function yesno = hasParameter(obj, p)
            % Returns true if the object has paramater P
            
            yesno = obj.Optimizer.agent_params.isKey(p);
        end
        
        function [u, info, openloop] = optimize(obj, x0)
            % Solves the optimization problem for a given initial condition
            
            narginchk(2, 2);
            assert(isequal(size(x0), [obj.Agent.nx, 1]), ...
                'The initial state must be a %dx1 vector.', obj.Agent.nx);
            [inputs, missing] = obj.readParameters();
            if any(missing)
                fprintf('Missing values of parameters:\n');
                fprintf('-----------------------------\n');
                obj.listMissing();
                fprintf('\n');
                error('There are missing values.');
            end
            [out, info] = obj.Optimizer.opt{[{x0}, inputs]};
            u = out{1};
            if nargout==3
                % keep the ordering consistnent with obj.Optimizer.outputs
                openloop.X = out{2};
                openloop.U = out{3};
                openloop.Y = out{4};
            end
        end
		
        function params = getParameters(obj)
            % Returns list of parametric variables in the model
            
            params = obj.Agent.getParameters('Agent.');
            for i = 1:length(obj.Obstacles)
                op = obj.Obstacles(i).getParameters(sprintf('Obstacles(%d).', i));
                params = [params; op]; % merge maps together
            end
        end

    end
        
    methods(Static, Hidden)
        
        function info = signal_info(signal_label, signal_data)
            % Splits strings like Obstacles(1).position(2).name into
            % components
            
            [~, ~, ~, n] = regexp(signal_label, '\w+');
            if length(n)==3
                % module.signal.property
                info.module = n{1};
                info.module_index = 1;
                info.signal = n{2};
                info.property = n{3};
            else
                info.module = n{1};
                info.module_index = str2double(n{2});
                info.signal = n{3};
                info.property = n{4};
            end
            info.dim = signal_data.Dim;
        end
        
    end
    
end
