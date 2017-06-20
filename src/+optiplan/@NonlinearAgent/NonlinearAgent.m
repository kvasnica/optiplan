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
        StateEq % state-update function x(k+1) = f(x(k), u(k), agent)
        OutputEq % output function: y(k) = g(x(k), u(k), agent)
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
            if nargin(value)~=3
                error('The function must take 3 inputs: x(k), u(k), agent');
            end
            obj.StateEq = value;
        end
        
        function set.OutputEq(obj, value)
            assert(isa(value, 'function_handle'), 'The value must be a function handle.');
            if nargin(value)~=3
                error('The function must take 3 inputs: x(k), u(k), agent');
            end
            obj.OutputEq = value;
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
                    cons = cons + [ x(:, :, k+1) == obj.StateEq(x(:, :, k), ...
                        u(:, :, k), obj) ];
                end
                if ~isempty(obj.OutputEq) && obj.ny > 0
                    cons = cons + [ y(:, :, k) == obj.OutputEq(x(:, :, k), ...
                        u(:, :, k), obj)];
                end
            end
        end
        
        function out = linearize(obj, X, U)
            % Linearizes nonlinear system around a trajectory
            %
            %   agent.linearize(Xs, Us)
            
            narginchk(3, 3);
            assert(size(X, 1)==obj.nx, '"X" must have %d row(s).', obj.nx);
            assert(size(U, 1)==obj.nu, '"U" must have %u row(s).', obj.nu);
            
            if ~isfield(obj.Internal, 'LinearizationFuns')
                % requires symbolic toolbox
                x = sym('x', [obj.nx 1]);
                u = sym('u', [obj.nu 1]);
                
                fx = matlabFunction(jacobian(obj.StateEq(x, u), x), 'Vars', {x, u});
                fu = matlabFunction(jacobian(obj.StateEq(x, u), u), 'Vars', {x, u});
                gx = matlabFunction(jacobian(obj.OutputEq(x, u), x), 'Vars', {x, u});
                gu = matlabFunction(jacobian(obj.OutputEq(x, u), u), 'Vars', {x, u});
                obj.Internal.LinearizationFuns = struct('fx', fx, 'fu', fu, ...
                    'gx', gx, 'gu', gu);
            end
            % pad trajectories to correct size
            out = struct('A', [], 'B', [], 'f', [], 'C', [], 'D', [], 'g', []);
            funs = obj.Internal.LinearizationFuns;
            for i = 1:min(size(U, 2), size(X, 2))
                % linearization points
                xs = X(:, i);
                us = U(:, i);
                if ~isempty(obj.StateEq)
                    out.A = [ out.A, funs.fx(xs, us) ];
                    out.B = [ out.B, funs.fu(xs, us) ];
                    out.f = [ out.f, obj.StateEq(xs, us)-funs.fx(xs, us)*xs-funs.fu(xs, us)*us ];
                end
                if ~isempty(obj.OutputEq)
                    out.C = [ out.C, funs.gx(xs, us) ];
                    out.D = [ out.D, funs.gu(xs, us) ];
                    out.g = [ out.g, obj.OutputEq(xs, us)-funs.gx(xs, us)*xs-funs.gu(xs, us)*us ];
                end
            end
        end

    end

end
