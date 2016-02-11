classdef AgentSignal < optiplan.utils.FilterBehavior & optiplan.utils.IterableBehavior
    %Object representation of a YALMIP variable with some properties

    % Copyright is with the following author(s):
    %
    % (C) 2016 Michal Kvasnica, Slovak University of Technology in Bratislava
    %          michal.kvasnica@stuba.sk
    %
    % This project is covered by the GNU GPL2 license. See COPYING for more
    % information.
    
    properties(Hidden)
        Name = '' % Name of the signal
    end
    properties(Hidden)
        UserData % User-defined value
    end
    properties(SetAccess = protected, Hidden)
        Var % YALMIP variable representing the signal
        Dim = [0 0] % Dimension of the signal
        N
	end
	
    methods
        
        function obj = AgentSignal(dim, N, name)
            % constructor:
            %   s = AgentSignal(n, m)
            %           n: number of variables
            %           m: number of signals (m=1 by default)

            assert(length(dim)==2, 'The first input must be a 1x2 vector.');
            obj.Dim = dim;
            obj.N = N;
            if nargin==3
                obj.Name = name;
            end

            % Default properties:
            %   .min (vector/matrix, constant, preview)
            %   .max
            %   .reference
            %   .penalty (matrix)
            % Optional properties:
            %   .value
            % TODO:
            %   .norm (scalar)
            %   .softening
            %   .penalty (preview)
			obj.with('Min'); % -Inf by default
			obj.with('Max'); % +Inf by default
			obj.with('Reference'); % zero by default
            obj.with('Penalty'); % zero by default
        end

        function out = length(obj)
            out = obj.N;
        end
        
        function out = size(obj)
            out = [obj.Dim(1) obj.Dim(2) obj.N];
        end
        
		function s = saveobj(obj)
			% save method
            
            disp('Saving of AgentSignal objects is not yet implemented.');
            return
            
			% we need to work on a copy of the object, since we are going
			% to remove from it filters and sdpvar objects
			s = copy(obj);
			s.saveSdpvarValue();
			% remove the sdpvar
			s.uninstantiate();
			s.saveAllFilters();
        end
        
		function uninstantiate(obj)
			% removes YALMIP's representation of the signal
			
            assert(obj.is_instantiated(), 'Signal is not instantiated.');
            % notify filters that they should remove any
            % self-introduced YALMIP variables
            obj.applyFilters('uninstantiate');
            obj.Var = [];
		end
		
        function instantiate(obj)
            % instantiate yalmip variable
            
            obj.Var = sdpvar(obj.Dim(1), obj.Dim(2), obj.N, 'full');
            % tell filters to instantiate their variables
            obj.applyFilters('instantiate');
        end

        function C = constraints(obj)
            % Convert variable into YALMIP constraints

            assert(obj.is_instantiated(), 'Signal is not instantiated.');
            % add constraints from filters
            C = obj.applyFilters('constraints');
        end
        
        function J = objective(obj)
            % Convert variable into objective

            assert(obj.is_instantiated(), 'Signal is not instantiated.');
            % add objective from filters
            J = obj.applyFilters('objective');
        end
        
        function out = double(obj, k)
            % Returns optimized value of the signal
            
            
            if nargin==2
                out = value(obj.Var(:, :, k));
            else
                out = [];
                for k = 1:obj.N
                    out = [out, value(obj.Var(:, :, k))];
                end
            end
        end

        function optimize(obj, sdpopts)
            % optimizes the signal
            
            if nargin<2
                sdpopts = sdpsettings('verbose', 0);
            end
            assert(obj.is_instantiated(), 'Signal is not instantiated.');
            C = obj.constraints();
            J = obj.objective();
            optimize(C, J, sdpopts);
        end
        
        function [params, names, variables] = getParameters(obj, prefix)
            % returns parametric variables of this signal
            
            if nargin<2
                prefix = '';
            end
            params = containers.Map;
            names = {};
            variables = {};
            
            map = obj.applyFilters('getVariables', 'map');
            keys = map.keys;
            for j = 1:length(keys)
                n = keys{j};
                if ~isempty(map(n))
                    if ~isempty(obj.Name)
                        nn = [obj.Name '.' n];
                    else
                        nn = n;
                    end
                    names{end+1} = [prefix nn];
                    variables{end+1} = map(n).Var;
                    params(names{end}) = map(n);
                end
            end
        end
        
        function [opt, inputorder] = getOptimizer(obj, solver, sdpopts)
            % returns optimizer for this signal

            narginchk(2, Inf);
            assert(obj.is_instantiated(), 'Signal is not instantiated.');
            if nargin<3
                sdpopts = sdpsettings('verbose', 0);
            end
            options = sdpsettings(sdpopts, 'solver', solver);
            C = obj.constraints();
            J = obj.objective();
            [~, inputorder, inputs] = obj.getParameters();
            opt = optimizer(C, J, options, inputs, obj.Var);
        end
    end
	
    methods(Access = protected)
        
        function yn = is_instantiated(obj)
            % Returns true iff the signal was instantiated
            yn = isa(obj.Var, 'ndsdpvar') || isa(obj.Var, 'sdpvar');
        end
        
        function out = default_getVariables_handler(obj, paramname, paramdim)
            % default handler for the getVariables callback
            %
            % Response: structure (or an array of structures) with following fields:
            %
            %  .Var: sdpvar representation of the introduced variable
            %  .Dim: dimension of the variable
            %  .Parametric: logical, if true, the variable will become part of the
            %              vector of initial conditions
            
            if nargin<3
                paramdim = obj.Dim;
            end
            if isequal(obj.(paramname), 'parameter')
                % do we have that variables stored as an sdpvar?
                if isfield(obj.Internal, 'parameters') && isfield(obj.Internal.parameters, paramname)
                    % yep, use that one
                    out.Var = obj.Internal.parameters.(paramname);
                    out.Dim = size(out.Var);
                else
                    % no sdpvar here yet
                    out.Var = [];
                    out.Dim = paramdim;
                end
                out.Parametric = true;
            else
                out = [];
            end
        end
        
        function out = default_instantiate_handler(obj, paramname, paramdim)
           % default handler for the instantiate callback
           
           if nargin<3
                paramdim = obj.Dim;
            end
           if isequal(obj.(paramname), 'parameter')
               obj.Internal.parameters.(paramname) = sdpvar(paramdim(1), paramdim(2), obj.N, 'full');
           end
           out = [];
        end
        
        function out = default_uninstantiate_handler(obj, paramname)
            % default handler for the uninstantiate callback
           
            obj.Internal.parameters.(paramname) = [];
            out = [];
        end
        
        function msg = check_set_value(obj, value)
            % returns a non-empty message if value is of wrong size/type
            
            msg = '';
            if isa(value, 'double')
                % dimensions must match if input is a double
                if ~isequal(size(value), obj.Dim)
                    % TODO: print either "vector" or "matrix"
                    msg = sprintf('Value must be a %dx%d vector/matrix.', obj.Dim(1), obj.Dim(2));
                end
            elseif ischar(value) && isequal(value, 'parameter')
                % all ok
            else
                msg = 'Unsupported value. Can only be double of ''parameter''.';
            end
        end
    end
    
	methods(Hidden)
        % private APIs
        
        function out = value_or_var(obj, k)
            % returns either sdpvar if value='parameter' or value otherwise
            
            out = obj.Value;
            if isequal(out, 'parameter')
                % use the signal's variable if it's parametric
                out = obj.Var;
                if nargin==2
                    out = out(:, :, k);
                end
            end
        end
        
		function msg = validatePenalty(obj, P)
			% validates penalty P

            % TODO: support non-square penalties for 1/Inf norms
            if ~isequal(size(P), [obj.Dim(1), obj.Dim(1)])
                msg = sprintf('The weighting matrix must be %dx%d.', obj.Dim(1), obj.Dim(1));
            elseif min(eig(P)) < 0
                msg = 'The weighting matrix must be positive semi-definite.';
            else
                msg = '';
            end
		end
    end
end
            
