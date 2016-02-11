classdef NonlinearAgent < optiplan.Agent
    % Class representing agents with linear dynamics.

    % Copyright is with the following author(s):
    %
    % (C) 2016 Michal Kvasnica, Slovak University of Technology in Bratislava
    %          michal.kvasnica@stuba.sk
    %
    % This project is covered by the GNU GPL2 license. See COPYING for more
    % information.

    properties
        StateEq % state-update function x(k+1) = f(x(k), u(k))
        OutputEq % output function: y(k) = g(x(k), u(k))
        ConstraintsFun % function of constraints cons(x(k), u(k), y(k))
        ObjectiveFun % objective function obj(x(k), u(k), y(k))
    end

    methods

        function obj = NonlinearAgent(varargin)
            % Constructor
            %
            %   a = Agent('nx', nx, 'nu', nu, 'ny', ny, 'PredictionHorizon', N)

            obj = obj@optiplan.Agent(varargin{:});
        end

        function set.StateEq(obj, value)
            assert(isa(value, 'function_handle'), 'The value must be a function handle.');
            obj.StateEq = value;
        end
        function set.OutputEq(obj, value)
            assert(isa(value, 'function_handle'), 'The value must be a function handle.');
            obj.OutputEq = value;
        end
        function set.ConstraintsFun(obj, value)
            assert(isa(value, 'function_handle'), 'The value must be a function handle.');
            obj.ConstraintsFun = value;
        end
        function set.ObjectiveFun(obj, value)
            assert(isa(value, 'function_handle'), 'The value must be a function handle.');
            obj.ObjectiveFun = value;
        end

        function out = objective(obj)
            % Returns YALMIP representation of the objective function
            
            out = objective@optiplan.Agent(obj);
            % add custom nonlinear objective
            x = obj.X.Var;
            u = obj.U.Var;
            y = obj.Y.Var;
            if ~isempty(obj.ObjectiveFun)
                for k = 1:obj.N
                    out = out + obj.ObjectiveFun(x(:, :, k), u(:, :, k), y(:, :, k));
                end
            end
        end
        
        function cons = constraints(obj)
            % Creates YALMIP constraints representing prediction equations
            %

            % add master constraints
            cons = constraints@optiplan.Agent(obj);
            
            % add nonlinear dynamics constraints
            x = obj.X.Var;
            u = obj.U.Var;
            y = obj.Y.Var;
            for k = 1:obj.N
                if ~isempty(obj.StateEq) && obj.nx > 0
                    cons = cons + [ x(:, :, k+1) == obj.StateEq(x(:, :, k), u(:, :, k)) ];
                end
                if ~isempty(obj.OutputEq) && obj.ny > 0
                    cons = cons + [ y(:, :, k) == obj.OutputEq(x(:, :, k), u(:, :, k))];
                end
            end
            
            % add custom nonlinear constraints
            if ~isempty(obj.ConstraintsFun)
                for k = 1:obj.N
                    cons = cons + obj.ConstraitnsFun(x(:, :, k), u(:, :, k), y(:, :, k));
                end
            end
        end
    end

end
