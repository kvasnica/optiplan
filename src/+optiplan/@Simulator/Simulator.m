classdef Simulator < optiplan.utils.OMPBaseClass
    % Class representing Planner-based simulators
    %
    %   psim = Simulator(planner)

    % Copyright is with the following author(s):
    %
    % (C) 2016 Michal Kvasnica, Slovak University of Technology in Bratislava
    %          michal.kvasnica@stuba.sk
    %
    % This project is covered by the GNU GPL2 license. See COPYING for more
    % information.

    properties(SetAccess=protected, Hidden)
        Planner
    end
    properties
        Results
        Parameters
    end
    
    methods
        
        function obj = Simulator(planner)
            % Constructor for Simulator objects
            %
            %   psim = Simulator(planner)
            
            assert(isa(planner, 'optiplan.Planner'), 'The first input must be a Planner object.');
            obj.Planner = copy(planner);
            obj.Parameters = planner.Parameters;
            obj.Results.X = [];
            obj.Results.U = [];
            obj.Results.Y = [];
            obj.Results.Predictions = [];
        end

        function run(obj, x0, Nsim, varargin)
            % Runs a closed-loop simulation
            %
            %   psim.run(x0, Nsim, 'param1', value1, 'param2, value2, ...)
            %
            % Inputs:
            %          x0 -- initial state (mandatory)
            %        Nsim -- number of simulation steps (mandatory)
            % Parameters:
            %   'Preview' -- true/false whether future references are
            %                provided to the optimization. only applies to
            %                signals which are marked as 'parameter'.
            %                default value is true
            %   'StateEq' -- function handle for a state-update equation.
            %                it must take current state "x" and current
            %                control action "u" as inputs and must generate
            %                the successor state as the output
            %  'OutputEq' -- function handle for an output equation.
            %                it must take current state "x" and current
            %                control action "u" as inputs and must generate
            %                the plant output "y" as the output
            %  'RadarDetector' -- function handle. the function must take
            %                     three inputs:
            %                       * agent's position
            %                       * position of the obstacle
            %                       * size of the obstacle
            %                     and must return true/false depending on
            %                     whether the obstacle can be seen from the
            %                     agent. see "omp_demo7.m" for a demo.
            %   'WaitBar' -- whether to display the progress bar 
            %                (true by default)
            %
            % After the simulation is ran, the results are stored in the
            % psim.Results structure as follows:
            %   psim.Results.X -- closed-loop profiles of states
            %   psim.Results.U -- closed-loop profiles of control actions
            %   psim.Results.Y -- closed-loop profiles of plant's outputs
            %
            % Additionally, open-loop predictions obtained at each step of
            % the simulation are stored in psim.Results.Predictions.
            %
            % Before the simulation is started, profiles of parametric
            % variables need to be specified in respective fields of the
            % psim.Parameters structure. Either provide the outlook of the
            % respective signals over Nsim steps, or just give a single
            % value. In the latter case, the value will be repeated over
            % the whole simulation horizon, given by Nsim. As an example,
            % psim.Parameters.Agent.Y.Reference = [1; 2] will be
            % automatically expanded to repmat([1; 2], 1, Nsim). If the
            % signal is time-varying, provide its profile over the next
            % Nsim steps as a matrix with Nsim columns.
            
            assert(isequal(size(x0), [obj.Planner.Agent.nx, 1]), ...
                'x0 must be a %dx1 vector.', obj.Planner.Agent.nx);

            ip = inputParser;
            ip.addParamValue('Preview', true, @islogical);
            ip.addParamValue('StateEq', []);
            ip.addParamValue('OutputEq', []);
            ip.addParamValue('RadarDetector', []);
            ip.addParamValue('WaitBar', true);
            ip.parse(varargin{:});
            Options = ip.Results;
            
            if Options.WaitBar
                wb_handle = waitbar(0, 'Simulating...');
                onclp = onCleanup(@() delete(wb_handle));
            end
            
            % check for missing parameters
            [params, missing, missing_names] = obj.readParameters(Nsim);
            if any(missing)
                fprintf('Missing values of parameters:\n');
                fprintf('-----------------------------\n');
                obj.listMissing(inputname(1));
                fprintf('\n');
                error('There are missing values.');
            end

            % read dynamics
            if isa(obj.Planner.Agent, 'optiplan.LinearAgent')
                f = {'A', 'B', 'C', 'D', 'f', 'g'};
                for i = 1:length(f)
                    if obj.hasParameter(sprintf('Agent.%s.Value', f{i}))
                        % read from parameters (already expanded)
                        simdata.(f{i}) = params.Agent.(f{i}).Value;
                    else
                        % read from the agent
                        simdata.(f{i}) = obj.Planner.Agent.(f{i}).Value;
                        % expand
                        simdata.(f{i}) = repmat(simdata.(f{i}), 1, Nsim);
                    end
                end
            end
            
            obj.Results.X = x0;
            obj.Results.U = [];
            obj.Results.Y = [];
            obj.Results.Predictions = [];
            nx = obj.Planner.Agent.nx;
            nu = obj.Planner.Agent.nu;
            ny = obj.Planner.Agent.ny;
            param_info = obj.Planner.Optimizer.param_info;
            for k = 1:Nsim
                if Options.WaitBar
                    waitbar(k/Nsim, wb_handle);
                end
                x0 = obj.Results.X(:, end); % last known state
                % copy parameters from obj.Parameters to
                % obj.Planner.Parameters
                for i = 1:length(param_info)
                    pi = param_info{i};
                    value = params.(pi.module)(pi.module_index).(pi.signal).(pi.property);
                    % pad it with N+1 at the end
                    value = repmat(value, 1, 2); % TODO: too agressive, but simple
                    if Options.Preview
                        %value = value(:, ((k-1)*pi.dim(2)+1):(k*pi.dim(2)));
                        value = value(:, ((k-1)*pi.dim(2)+1):((k-1)*pi.dim(2)+pi.dim(2)*pi.dim(3)));
                    else
                        value = repmat(value(:, ((k-1)*pi.dim(2)+1):k*pi.dim(2)), 1, pi.dim(3));
                    end
                    obj.Planner.Parameters.(pi.module)(pi.module_index).(pi.signal).(pi.property) = value;
                end
                % radar detection
                if ~isempty(Options.RadarDetector)
                    for i = 1:length(obj.Planner.Obstacles)
                        if obj.hasParameter(sprintf('Obstacles(%d).Size.Value', i))
                            osize = obj.Parameters.Obstacles(i).Size.Value(:, k);
                        else
                            osize = obj.Planner.Obstacles(i).Size.Value(:, 1);
                        end
                        if obj.hasParameter(sprintf('Obstacles(%d).Position.Value', i))
                            opos = obj.Parameters.Obstacles(i).Position.Value(:, k);
                        else
                            opos = obj.Planner.Obstacles(i).Position.Value(:, 1);
                        end
                        if k>1
                            apos = obj.Results.Y(:, end);
                            cansee = Options.RadarDetector(apos, opos, osize);
                        else
                            % TODO: need a better initialization
                            cansee = 1;
                        end
                        obj.Planner.Parameters.Obstacles(i).Visible.Value = repmat(cansee, 1, obj.Planner.Agent.N);
                    end
                end

                % optimize
                [u, prob, openloop] = obj.Planner.optimize(x0);
                if prob~=0
                    error('Problem at step %d: %s', k, yalmiperror(prob));
                end
                % state update
                if ~isempty(Options.StateEq)
                    % user-provided nonlinear state-update equation
                    xn = Options.StateEq(x0, u);
                elseif isa(obj.Planner.Agent, 'optiplan.NonlinearAgent')
                    % nonlinear state update eq. from the agent
                    xn = obj.Planner.Agent.StateEq(x0, u, obj.Planner.Agent);
                else
                    % linear dynamics
                    xn = simdata.A(:, ((k-1)*nx+1):(k*nx))*x0+simdata.B(:, ((k-1)*nu+1):(k*nu))*u+simdata.f(:, k);
                end
                if ~isempty(Options.OutputEq)
                    % user-provided nonlinear output equation
                    y = Options.OutputEq(x0, u);
                elseif isa(obj.Planner.Agent, 'optiplan.NonlinearAgent')
                    % nonlinear output eq. from the agent
                    y = obj.Planner.Agent.OutputEq(x0, u, obj.Planner.Agent);
                else
                    % linear dynamics
                    y = simdata.C(:, ((k-1)*nx+1):(k*nx))*x0+simdata.D(:, ((k-1)*nu+1):(k*nu))*u+simdata.g(:, k);
                end
                obj.Results.X = [obj.Results.X, xn];
                obj.Results.Y = [obj.Results.Y, y];
                obj.Results.U = [obj.Results.U, u];
                if isempty(obj.Results.Predictions)
                    obj.Results.Predictions = openloop;
                else
                    obj.Results.Predictions(k) = openloop;
                end
                obj.Results.Nsim = Nsim;
            end
        end
        
        function plot(obj, varargin)
            % Visualize the closed-loop simulation
            
            assert(~isempty(obj.Results.X), 'Run the simulation first.');
            Nsim = obj.Results.Nsim;
            N = obj.Planner.Agent.N;
            
            ip = inputParser;
            ip.addParamValue('Axis', []);
            ip.addParamValue('AxisType', ''); % square, equal, auto, tight
            ip.addParamValue('Grid', true);
            ip.addParamValue('Trail', false);
            ip.addParamValue('TrailColor', 'm');
            ip.addParamValue('TrailStyle', '-');
            ip.addParamValue('TrailWidth', 1);
            ip.addParamValue('Constraints', true);
            ip.addParamValue('ConstraintsColor', 'k');
            ip.addParamValue('ConstraintsStyle', '-');
            ip.addParamValue('ConstraintsWidth', 3);
            ip.addParamValue('AgentColor', 'g');
            ip.addParamValue('AgentMinSize', []);
            ip.addParamValue('AgentPredictionColor', 'k');
            ip.addParamValue('Reference', false);
            ip.addParamValue('ReferenceColor', 'r');
            ip.addParamValue('ReferenceStyle', ':');
            ip.addParamValue('ReferenceWidth', 1);
            ip.addParamValue('Delay', 0.05);
            ip.addParamValue('Predictions', false);
            ip.addParamValue('PredSteps', obj.Planner.Agent.N);
            ip.addParamValue('Previous', []);
            ip.addParamValue('PreviousColor', 'b');
            ip.addParamValue('PreviousStyle', '--');
            ip.addParamValue('PreviousWidth', 1);
            ip.addParamValue('ObstacleColor', 'y');
            ip.addParamValue('RadarDetector', []);
            ip.addParamValue('RadarPlotter', []);
            ip.addParamValue('Loops', 1);
            ip.parse(varargin{:});
            Options = ip.Results;
            
            % extract data either from obj.Parameters or from the
            % agent/obstacles
            %
            % TODO: automate this
            params = [];
            params.Obstacles = [];
            if obj.hasParameter('Agent.Y.Reference')
                params.Agent.Y.Reference = repmat(obj.Parameters.Agent.Y.Reference, 1, Nsim+N+1);
            else
                params.Agent.Y.Reference = repmat(obj.Planner.Agent.Y.Reference, 1, Nsim+N+1);
            end
            if obj.hasParameter('Agent.Y.Max')
                params.Agent.Y.Max = obj.Parameters.Agent.Y.Max;
            else
                params.Agent.Y.Max = repmat(obj.Planner.Agent.Y.Max, 1, Nsim+N+1);
            end
            if obj.hasParameter('Agent.Y.Min')
                params.Agent.Y.Min = obj.Parameters.Agent.Y.Min;
            else
                params.Agent.Y.Min = repmat(obj.Planner.Agent.Y.Min, 1, Nsim+N+1);
            end
            if obj.hasParameter('Agent.Size.Value')
                params.Agent.Size = obj.Parameters.Agent.Size.Value;
            else
                params.Agent.Size = repmat(obj.Planner.Agent.Size.Value, 1, Nsim+N+1);
            end
            for i = 1:length(obj.Planner.Obstacles)
                if obj.hasParameter(sprintf('Obstacles(%d).Size.Value', i))
                    params.Obstacles(i).Size = obj.Parameters.Obstacles(i).Size.Value;
                else
                    params.Obstacles(i).Size = repmat(obj.Planner.Obstacles(i).Size.Value, 1, Nsim+N+1);
                end
                if obj.hasParameter(sprintf('Obstacles(%d).Position.Value', i))
                    params.Obstacles(i).Position = obj.Parameters.Obstacles(i).Position.Value;
                else
                    params.Obstacles(i).Position = repmat(obj.Planner.Obstacles(i).Position.Value, 1, Nsim+N+1);
                end
            end
            if isempty(Options.AgentMinSize)
                for i = 1:size(params.Agent.Size, 2)
                    if nnz(params.Agent.Size(:, i))==0
                        error('Options.AgentMinSize must be set.');
                    end
                end
            end
            % determine default axis
            if isempty(Options.Axis)
                ymax = max(params.Agent.Y.Max, [], 2);
                ymin = min(params.Agent.Y.Min, [], 2);
                d = (ymax-ymin)/10;
                ax = [];
                for i = 1:length(ymax)
                    ax = [ax, ymin(i)-d(i) ymax(i)+d(i)];
                end
                Options.Axis = ax;
            end
            
            % main figure
            scenefig = figure;
            hold on
            axis(Options.Axis);
            if ~isempty(Options.AxisType)
                axis(Options.AxisType);
            end
            if Options.Grid
                grid on
            end
            
            % plot position reference if desired
            if Options.Reference
                plot(params.Agent.Y.Reference(1, 1:Nsim), ...
                    params.Agent.Y.Reference(2, 1:Nsim), ...
                    'color', Options.ReferenceColor, ...
                    'linestyle', Options.ReferenceStyle, ...
                    'linewidth', Options.ReferenceWidth);
            end
            
            % plot previous trajectory if provided
            if ~isempty(Options.Previous)
                plot(Options.Previous(1, :), ...
                    Options.Previous(2, :), ...
                    'color', Options.PreviousColor, ...
                    'linestyle', Options.PreviousStyle, ...
                    'linewidth', Options.PreviousWidth);
            end
            
            for iloop = 1:Options.Loops
            for k = 1:Nsim
                % plot the trail (stays on the plot)
                if Options.Trail && k>1
                    plot([obj.Results.Y(1, k-1); obj.Results.Y(1, k)], ...
                        [obj.Results.Y(2, k-1); obj.Results.Y(2, k)], ...
                        'color', Options.TrailColor, ...
                        'linestyle', Options.TrailStyle, ...
                        'linewidth', Options.TrailWidth);
                end
                
                handles = [];
                
                % plot constraints
                if Options.Constraints && ...
                        all(isfinite(params.Agent.Y.Min(:, k))) && ...
                        all(isfinite(params.Agent.Y.Max(:, k)))
                    cpos = [params.Agent.Y.Min(1, k), params.Agent.Y.Min(2, k), ...
                        params.Agent.Y.Max(1, k)-params.Agent.Y.Min(1, k), ...
                        params.Agent.Y.Max(2, k)-params.Agent.Y.Min(2, k)];
                    h = rectangle('Position', cpos, ...
                        'EdgeColor', Options.ConstraintsColor, ...
                        'LineStyle', Options.ConstraintsStyle, ...
                        'LineWidth', Options.ConstraintsWidth);
                    handles = [handles, h];
                end
                
                % plot obstacles
                for i = 1:length(params.Obstacles)
                    if ~isempty(Options.RadarDetector)
                        % can the obstacle be seen?
                        apos = obj.Results.Y(:, k);
                        opos = params.Obstacles(i).Position(:, k);
                        osize = params.Obstacles(i).Size(:, k);
                        cansee = Options.RadarDetector(apos, opos, osize);
                    elseif ~isfield(params.Obstacles(i), 'Visible')
                        cansee = obj.Planner.Obstacles(i).Visible.Value(1);
                    else
                        cansee = true;
                    end
                    h = optiplan.utils.plotRectangle('Position', params.Obstacles(i).Position(:, k), ...
                        'Size', params.Obstacles(i).Size(:, k), ...
                        'Color', Options.ObstacleColor, ...
                        'Wire', ~cansee);
                    handles = [handles, h];
                end
                
                % plot predictions if desired
                if Options.Predictions
                    pred_range = 2:obj.Planner.Agent.N;
                    pred_range = round(linspace(2, obj.Planner.Agent.N, Options.PredSteps));
                    for i = pred_range
                        % plot the predicted position of the agent
                        pos = obj.Results.Predictions(k).Y;
                        h = optiplan.utils.plotRectangle('Position', obj.Results.Predictions(k).Y(:, i), ...
                            'Size', params.Agent.Size(:, k+i-1), ...
                            'MinSize', Options.AgentMinSize, ...
                            'Color', Options.AgentPredictionColor, ...
                            'Wire', true);
                        handles = [handles, h];
                        % plot the previewed references
                        h = plot(params.Agent.Y.Reference(1, k+i), ...
                            params.Agent.Y.Reference(2, k+i), ...
                            'color', Options.ReferenceColor, ...
                            'marker', 'o', 'markersize', 10);
                        handles = [handles, h];
                    end
                end

                % plot the agent
                % TODO: support rotation
                % TODO: support more shapes
                h = optiplan.utils.plotRectangle('Position', obj.Results.Y(:, k), ...
                    'Size', params.Agent.Size(:, k), ...
                    'MinSize', Options.AgentMinSize, ...
                    'Color', Options.AgentColor);
                handles = [handles, h];

                % plot the current reference
                h = plot(params.Agent.Y.Reference(1, k), ...
                    params.Agent.Y.Reference(2, k), ...
                    'color', Options.ReferenceColor, ...
                    'marker', 'o', 'markersize', 10);
                handles = [handles, h];

                % plot the radar if requested
                if ~isempty(Options.RadarPlotter)
                    h = Options.RadarPlotter(obj.Results.Y(:, k));
                    handles = [handles, h];
                end

                title(sprintf('Simulation step: %d/%d', k, Nsim));
                pause(Options.Delay);
                if k < Nsim
                    delete(handles);
                end
            end
            end
            hold off
        end
        
        function [params, missing, missing_names] = readParameters(obj, Nsim)
            % Prepares inputs for the optimizer
            
            if nargin<2
                Nsim = 1;
            end
            % this function needs to be as efficient as possible
            names = obj.Planner.Optimizer.parameters;
            infos = obj.Planner.Optimizer.param_info;
            n = length(names);
            missing = false(1, n);
            params = obj.Parameters;
            for i = 1:n
                pi = infos{i};
                value = obj.Parameters.(pi.module)(pi.module_index).(pi.signal).(pi.property);
                if isequal(value, 'parameter') || isempty(value) || all(all(isnan(value)))
                    value = [];
                    missing(i) = true;
                elseif size(value, 2)==pi.dim(2)
                    % automatically expand the value to horizon-N
                    value = repmat(value, 1, Nsim);
                    params.(pi.module)(pi.module_index).(pi.signal).(pi.property) = value;
                elseif Nsim>0 && size(value, 2)<pi.dim(2)*Nsim
                    error('Parameters.%s(%d).%s.%s must be either a %dx%d or a %dx%d matrix.', ...
                        pi.module, pi.module_index, pi.signal, pi.property, ...
                        pi.dim(1), pi.dim(2), ...
                        pi.dim(1), pi.dim(2)*Nsim);
                end
            end
            missing_names = obj.Planner.Optimizer.parameters(missing);
        end

        function listParameters(obj)
            % Returns/displays parameters of the object

            obj.Planner.listParameters();
        end
        
        function yesno = hasParameter(obj, p)
            % Returns true if the object has paramater P
            
            yesno = obj.Planner.hasParameter(p);
        end

        function k = listMissing(obj, prefix)
            % Returns list of parameters that have not been set
            
            if nargin<2
                prefix = inputname(1);
            end
            [~, missing] = obj.readParameters(0);
            % recreate the parameter map
            k = obj.Planner.Optimizer.agent_params.keys;
            v = obj.Planner.Optimizer.agent_params.values;
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
                obj.Planner.Agent.listParameters(prefix, containers.Map(k, v));
                clear k
            end
        end
		
        function params = getParameters(obj)
            % Returns list of parametric variables in the model
            params = obj.Planner.getParameters();
        end

    end
    
    methods(Static)
        function T = circularTrajectory(Nsim, varargin)
            % Circular trajectory of a given radius and frequency
            %
            %    traj = circularTrajectory(Nsim, 'Radius', r, 'Loops', 1)
            
            ip = inputParser;
            ip.addParamValue('Radius', 0);
            ip.addParamValue('Loops', 1);
            ip.parse(varargin{:});
            Options = ip.Results;
            
            phi = linspace(-pi, pi, ceil(Nsim/Options.Loops)+1);
            T = Options.Radius*[cos(phi); sin(phi)];
            T = repmat(T, 1, Options.Loops);
            T = T(:, 1:Nsim);
        end
        function T = pointwiseTrajectory(Nsim, waypoints)
            % Piecewise trajectory connecting given waypoints

            T = [];
            npoints = size(waypoints, 2);
            for i = 1:npoints
                T = [T, repmat(waypoints(:, i), 1, ceil(Nsim/npoints))];
            end
            T = T(:, 1:Nsim);
        end
        function cansee = circularRadar(RadarRadius, apos, opos, osize)
            % Returns true if an obstacle is in a circular neighborhood of
            % the agent

            % is there an intersection between a circle with radius R,
            % centered at APOS, and a rectangle centered at OPOS and size
            % OSIZE?
            %
            % http://stackoverflow.com/questions/401847/circle-rectangle-collision-detection-intersection

            cdist = abs(apos - opos);
            if any(cdist > (osize/2+RadarRadius))
                cansee = false;
            elseif any(cdist <= osize/2)
                cansee = true;
            else
                cansee = norm(cdist-osize/2) <= RadarRadius;
            end
        end
    
    end
        
end
