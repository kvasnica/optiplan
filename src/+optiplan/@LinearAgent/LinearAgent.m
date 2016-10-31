classdef LinearAgent < optiplan.Agent
    % Class representing agents with linear dynamics.

    % Copyright is with the following author(s):
    %
    % (C) 2016 Michal Kvasnica, Slovak University of Technology in Bratislava
    %          michal.kvasnica@stuba.sk
    %
    % This project is covered by the GNU GPL2 license. See COPYING for more
    % information.

    properties
        % matrices of linear dynamics:
        %   x(k+1) = A*x(k) + B*u(k) + f
        %     y(k) = C*x(k) + D*u(k) + g
        A
        B
        f
        C
        D
        g
    end

    methods

        function obj = LinearAgent(varargin)
            % Constructor
            %
            %   a = Agent('nx', nx, 'nu', nu, 'ny', ny, 'PredictionHorizon', N)

            obj = obj@optiplan.Agent(varargin{:});
            
            vars = {'A', 'B', 'f', 'C', 'D', 'g'};
            dims = {[obj.nx, obj.nx], [obj.nx, obj.nu], [obj.nx, 1], ...
                [obj.ny, obj.nx], [obj.ny, obj.nu], [obj.ny, 1]};
            for i = 1:length(vars)
                s = optiplan.AgentSignal(dims{i}, obj.N, vars{i});
                % remove all filters
                f = s(1).listFilters();
                s.forEach(@(e) e.without(f));
                % add the "value" filter
                s.forEach(@(e) e.with('Value'));
                % add the signal to the agent
                obj.(vars{i}) = s;
            end
        end

        function cons = constraints(obj)
            %
            % Creates YALMIP constraints representing prediction equations
            %

            % add master constraints
            cons = constraints@optiplan.Agent(obj);
            
            x = obj.X.Var;
            y = obj.Y.Var;
            u = obj.U.Var;
            
            % add linear dynamics constraints
            for k = 1:obj.N
                % get the dynamics at the k-th step
                fields = {'A', 'B', 'f', 'C', 'D', 'g'};
                dyn = [];
                for i = 1:length(fields)
                    fld = fields{i};
                    dyn.(fld) = obj.(fld).value_or_var(k);
                end
                if obj.nx > 0
                    cons = cons + [ x(:, :, k+1) == dyn.A*x(:, :, k) + ...
                        dyn.B*u(:, :, k) + dyn.f];
                end
                if obj.ny > 0
                    cons = cons + [ y(:, :, k) == dyn.C*x(:, :, k) + ...
                        dyn.D*u(:, :, k) + dyn.g];
                end
            end
        end
    end

    methods(Static)
        function mpcoptions = demo2Ddata(Ts)
            % dynamics of the Demo2D agent
            %
            % see "help optiplan.LinearAgent.demo2D
            
            A = [1 0; Ts 1]; B = [Ts; 0.5*Ts^2]; C = [0 1];
            mpcoptions.A = [A, zeros(2); zeros(2) A];
            mpcoptions.B = [B zeros(2, 1); zeros(2, 1) B];
            mpcoptions.f = zeros(4, 1);
            mpcoptions.C = [C zeros(1, 2); zeros(1, 2) C];
            mpcoptions.D = zeros(2, 2);
            mpcoptions.g = zeros(2, 1);
            mpcoptions.Qy = eye(2); mpcoptions.Qu = eye(2);
            mpcoptions.umin = [-2; -2]; % minimal accelerations in x/y axis
            mpcoptions.umax = [2; 2]; % maximal accelerations in x/y axis
            mpcoptions.ymin = [-19; -19]; % minimal positions in x/y axis
            mpcoptions.ymax = [19; 19]; % maximal positions in x/y axis
            vmin = [-2; -2]; % minimal speeds in x/y axis
            vmax = [2; 2]; % maximal speeds in x/y axis
            mpcoptions.xmin = [vmin(1); mpcoptions.ymin(1); vmin(2); mpcoptions.ymin(2)];
            mpcoptions.xmax = [vmax(1); mpcoptions.ymax(1); vmax(2); mpcoptions.ymax(2)];
        end

        function [agent, mpcoptions] = demo2D(varargin)
            % Generates a demo agent representing a vehicle moving in a 2D plane
            %
            %   agent = optiplan.AgentFactory.DemoAgent2D('PredictionHorizon', N, 'SamplingTime', Ts)
            %
            % The dynamics in each axis is driven, independently, by a
            % double-integrator dynamics of the form
            %
            %   z(k+1) = A*z(k) + B*v(k)
            %     p(k) = C*z(k)
            %
            % where "p" is the position of the vehicle in the corresponding axis, "z"
            % is the vehicles state (consisting of the speed and the position), and "v"
            % is the acceleration in the corresponding axis.
            %
            % The dynamics as well as constraints are assumed to be time-invariant and
            % known a-priori.
            %
            % The aggregated model has:
            %   2 inputs [ax; ay]  -- accelerations in the x- and y-axis
            %   2 outputs [px; py] -- positions in the x- and y-axis
            %   4 states [vx; px; vy; py] where "vx" and "vy" are the speeds in the
            %                             corresponding axis
            %
            % PredictionHorizon is a mandatory input.
            % SamplingTime is optional (default is 0.25)
            
            % Copyright is with the following author(s):
            %
            % (C) 2016 Michal Kvasnica, Slovak University of Technology in Bratislava
            %          michal.kvasnica@stuba.sk
            %
            % This project is covered by the GNU GPL2 license. See COPYING for more
            % information.
            
            if nargin==0
                help optiplan.LinearAgent.demo2D
                if nargout==1
                    agent = [];
                end
                return
            end
            
            % must get at least 'PredictionHorizon' key/value pair
            narginchk(2, Inf);
            ip = inputParser;
            ip.KeepUnmatched = true;
            ip.addParamValue('PredictionHorizon', 10, @isnumeric);
            ip.addParamValue('SamplingTime', 0.25, @isnumeric);
            ip.addParamValue('Dynamics', 'constant');
            ip.addParamValue('StateConstraints', 'constant');
            ip.addParamValue('InputConstraints', 'constant');
            ip.addParamValue('OutputConstraints', 'constant');
            ip.addParamValue('OutputReference', 'constant');
            ip.addParamValue('AgentSize', 'constant');
            ip.addParamValue('Obstacles', 0);
            ip.parse(varargin{:});
            options = ip.Results;
            
            % create a basic linear agent
            agent = optiplan.LinearAgent('nx', 4, 'nu', 2, 'ny', 2, ...
                'PredictionHorizon', options.PredictionHorizon);
            
            % sample dynamics/constraints
            mpcoptions = optiplan.LinearAgent.demo2Ddata(options.SamplingTime);
            
            % dynamics
            if isequal(options.Dynamics, 'constant')
                % constant dynamics
                agent.A.Value = mpcoptions.A;
                agent.B.Value = mpcoptions.B;
                agent.C.Value = mpcoptions.C;
                agent.D.Value = zeros(2, 2);
                agent.f.Value = zeros(4, 1);
                agent.g.Value = zeros(2, 1);
            end
            if isequal(options.StateConstraints, 'constant')
                % constant state constraints
                agent.X.Min = mpcoptions.xmin;
                agent.X.Max = mpcoptions.xmax;
            end
            if isequal(options.InputConstraints, 'constant')
                % constant input constraints
                agent.U.Min = mpcoptions.umin;
                agent.U.Max = mpcoptions.umax;
            end
            if isequal(options.OutputConstraints, 'constant')
                % constant output constraints
                agent.Y.Min = mpcoptions.ymin;
                agent.Y.Max = mpcoptions.ymax;
            end
            % always constant state/input references and penalties
            agent.X.Reference = zeros(agent.nx, 1);
            agent.X.Penalty = zeros(agent.nx);
            agent.U.Reference = zeros(agent.nu, 1);
            agent.U.Penalty = mpcoptions.Qu;
            % always constant output penalty
            agent.Y.Penalty = mpcoptions.Qy;
            if isequal(options.OutputReference, 'constant')
                % constant output reference
                agent.Y.Reference = zeros(agent.ny, 1);
            end
            if isequal(options.AgentSize, 'constant')
                % constant size of the agent
                agent_size = [0; 0]; % size of the box in both axis
                agent.Size.Value = agent_size;
            end
            for i = 1:options.Obstacles
                agent.addObstacle();
            end
        end

    end
    
end
