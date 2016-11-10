classdef LinearizedAgent < moantool.LinearAgent & moantool.NonlinearAgent
    % Class representing agents with linearized dynamics

    % Copyright is with the following author(s):
    %
    % (C) 2016 Michal Kvasnica, Slovak University of Technology in Bratislava
    %          michal.kvasnica@stuba.sk
    %
    % This project is covered by the GNU GPL2 license. See COPYING for more
    % information.

    methods

        function obj = LinearizedAgent(varargin)
            % Constructor
            %
            %   a = LinearizedAgent('nx', nx, 'nu', nu, 'ny', ny, 'PredictionHorizon', N)
            
            obj@moantool.LinearAgent(varargin{:});
            obj@moantool.NonlinearAgent(varargin{:});
        end
        
        function cons = constraints(obj)
            % Creates YALMIP constraints representing prediction equations
            %

            % only use the linearized dynamics
            cons = constraints@moantool.LinearAgent(obj);
        end

        function params = computeParameters(obj, params, X, U)
            % Compute parameters (e.g. linearization)
            
            lindyn = obj.linearize(X, U);
            params.A.Value = lindyn.A;
            params.B.Value = lindyn.B;
            params.f.Value = lindyn.f;
            params.C.Value = lindyn.C;
            params.D.Value = lindyn.D;
            params.g.Value = lindyn.g;
        end

    end
    
end
