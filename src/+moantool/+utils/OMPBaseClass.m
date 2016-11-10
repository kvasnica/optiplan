classdef OMPBaseClass < moantool.utils.FilterBehavior & moantool.utils.ComponentBehavior & moantool.utils.IterableBehavior & moantool.utils.MPTUIHandle
    % Base class from which Agent and Obstacle inherit

    % Copyright is with the following author(s):
    %
    % (C) 2016 Michal Kvasnica, Slovak University of Technology in Bratislava
    %          michal.kvasnica@stuba.sk
    %
    % This project is covered by the GNU GPL2 license. See COPYING for more
    % information.

    methods
        function params = getParameters(obj, prefix)
            % Returns list of parametric variables in the model

            if nargin<2
                prefix = '';
            end
            params = containers.Map;
            props = properties(obj);
            for i = 1:length(props)
                signal = obj.(props{i});
                if ismethod(signal, 'getParameters')
                    p = signal.getParameters(prefix);
                    if ~isempty(p)
                        % add parameters of this signal to the stack
                        k = p.keys;
                        for ik = 1:length(k)
                            params(k{ik}) = p(k{ik});
                        end
                    end
                end
            end
        end

        function cons = constraints(obj)
            %
            % Creates YALMIP constraints of the object
            %

            cons = [];
            for i = 1:length(obj)
                assert(obj(i).is_instantiated(), 'Object must be instantiated.');
                cons = cons + obj(i).applyFilters('constraints');
                cons = obj(i).apply_recursively('constraints', cons);
            end
        end

        function out = objective(obj)
            %
            % Creates YALMIP objective for the object
            %

            out = 0;
            for i = 1:length(obj)
                assert(obj(i).is_instantiated(), 'Object must be instantiated.');
                out = out + obj(i).applyFilters('objective');
                out = obj(i).apply_recursively('objective', out);
            end
        end

        function data = getOptimizerData(obj, prefix)
            % returns info about parametric inputs and optimized variables

            if nargin<2
                prefix = '';
            end
            assert(obj.is_instantiated(), 'Object must be instantiated.');
            params = obj.getParameters(prefix);
            % extract variables from the map
            data.inputorder = params.keys;
            data.inputs = {};
            for i = 1:length(data.inputorder)
                v = params(data.inputorder{i});
                % TODO: use shiftdim to collapse 3D arrays to 2D ones
                data.inputs{i} = [];
                for j = 1:size(v.Var, 3)
                    data.inputs{i} = [data.inputs{i}, v.Var(:, :, j)];
                end
            end
            data.outputs = {};
            data.outputorder = {};
        end

        function new = saveobj(obj)
            % save method

            disp('Saving of OMP objects is not supported.');
        end

        function instantiate(obj)
            %
            % Creates YALMIP variables representing individual variables
            %

            for i = 1:length(obj)
                % instantiate modules/signals
                obj(i).apply_recursively('instantiate');
                % only instantiate the system afterwards
                obj(i).applyFilters('instantiate');
                obj(i).Internal.instantiated = true;
            end
        end

        function obj = uninstantiate(obj)
            %
            % Removes the YALMIP's representation of signals
            %

            for i = 1:length(obj)
                obj(i).applyFilters('uninstantiate');
                obj(i).apply_recursively('uninstantiate');
                obj(i).Internal.instantiated = false;
            end
        end

        function out = is_instantiated(obj)
            % Returns true if all variables have been instantiated
            out = obj.Internal.instantiated;
        end

        function listParameters(obj, prefix, params)
            % Returns/displays parameters of the object

            if nargin<2
                objName = inputname(1);
                if isempty(objName)
                    objName = 'obj';
                end
                prefix = [objName '.'];
            end
            if nargin<3
                params = obj.getParameters();
            end
            if isempty(params)
                fprintf('The object has no parameters.\n');
                return
            end
            keys = params.keys;
            longestName = 5;
            for i = 1:length(keys)
                longestName = max(longestName, length(keys{i}));
            end
            % header = sprintf('Name%s Dimension', repmat(' ', 1, longestName+1-4+length(prefix)));
            % fprintf('%s\n%s\n', header, repmat('-', 1, length(header')));
            for i = 1:length(keys)
                v = params(keys{i});
                fprintf('%s%s  %s%s\n', prefix, keys{i}, ...
                    repmat(' ', 1, longestName-length(keys{i})), ...
                    mat2str(v.Dim));
            end
        end
        
        function params = computeParameters(obj, params, varargin)
            % Compute parameters (e.g. linearization)
            
            % does nothing unless overloaded
        end

    end

    methods(Hidden)
        % private APIs
        function modules = discover_modules(obj)
            % discover all modules (aka signal keepers) in the object

            p = properties(obj);
            modules = {};
            for i = 1:length(p)
                if isstruct(obj.(p{i}))
                    modules{end+1} = p{i};
                end
            end
        end
    end

    methods(Access = protected)

        function out = apply_recursively(obj, fun, out)
            % apply function FUN to all signals in the object

            % find signals
            props = properties(obj);
            for i = 1:length(props)
                signal = obj.(props{i});
                if ismethod(signal, fun)
                    % if function is applicable, apply it to each element
                    % of the signal
                    if nargin==3
                        z = feval(fun, signal);
                        out = out + z;
                    else
                        feval(fun, signal);
                    end
                end
            end
            %             % find modules
            %             modules = obj.discover_modules();
            %             for i = 1:length(modules)
            %                 % for each module array
            %                 mod = obj.(modules{i});
            %                 for m = 1:length(mod)
            %                     % for each element of the module array
            %                     vars = fieldnames(mod(m));
            %                     for j = 1:length(vars)
            %                         % for each signal
            %                         var = mod(m).(vars{j});
            %                         if ismethod(var, fun)
            %                             % if function is applicable
            %                             for k = 1:length(var)
            %                                 % apply it to each element of the signal
            %                                 if nargin==3
            %                                     z = feval(fun, var(k));
            %                                     out = out + z;
            %                                 else
            %                                     feval(fun, var(k));
            %                                 end
            %                             end
            %                         end
            %                     end
            %                 end
            %             end

        end

    end

end
