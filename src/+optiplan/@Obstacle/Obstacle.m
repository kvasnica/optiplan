classdef Obstacle < optiplan.utils.OMPBaseClass
    % Class representing rectangular obstacles

    % Copyright is with the following author(s):
    %
    % (C) 2016 Michal Kvasnica, Slovak University of Technology in Bratislava
    %          michal.kvasnica@stuba.sk
    %
    % This project is covered by the GNU GPL2 license. See COPYING for more
    % information.

    properties(SetAccess=protected)
        Position
        Size
        Visible
    end

    methods

        function obj = Obstacle(agent, nObstacles)
            % Constructor
            %
            %   obst = Obstacle(agent)

            if nargin==1
                nObstacles = 1;
            end
            
            if nObstacles>1
                obj = [];
                for i = 1:nObstacles
                    obj = [obj, optiplan.Obstacle(agent)];
                end
                return
            end
            
            % position of the obstacle
            obj.Position = optiplan.AgentSignal([agent.ny 1], agent.N, 'Position');
            % whether the obstacle is visible
            obj.Visible = optiplan.AgentSignal([1 1], agent.N, 'Visible');
            % size along the axis
            obj.Size = optiplan.AgentSignal([agent.ny 1], agent.N, 'Size');
            % remove all filters
            obj.Position.without(obj.Position.listFilters());
            obj.Visible.without(obj.Visible.listFilters());
            obj.Size.without(obj.Size.listFilters());
            % add the "value" filter
            obj.Position.with('Value');
            obj.Visible.with('Value');
            obj.Size.with('Value');

            obj.Internal.instantiated = false;
        end
        
        function cons = constraints(obj, agent, MinSeparation, BigBound)
            %
            % Creates YALMIP constraints of the object
            %
            
            % apply constraints on variables
            cons = constraints@optiplan.utils.OMPBaseClass(obj);

            % TODO: support named groups instead of hard-coded outputs
            ydim = agent.Y.Dim(1);
            for io = 1:length(obj)
                % for each obstacle
                for k = 1:length(obj(io).Position)
                    % for each step of the prediction horizon
                    y = agent.Y.Var(:, :, k);
                    cons = cons + [ -BigBound <= y(:) <= BigBound ];
                    % position, size & visibility can either be parameters
                    % or fixed
                    opos = obj(io).Position.value_or_var(k);
                    osize = obj(io).Size.value_or_var(k);
                    ovisible = obj(io).Visible.value_or_var(k);
                    % size of the agent
                    asize = agent.Size.value_or_var(k);
                    
                    if isequal(ovisible, 0)
                        % no obstacle here
                        continue
                    end
                    
                    % only rectangular obstacles are supported for now
                    % TODO: support nD obstacles
                    
                    % the obstacle is given by
                    % { y | -obstsize_i<=(y_{k,i}-obstpos_{k,i})<=obstsize_i }
                    %
                    % to avoid the obstacle, y_k has to violate at least one constraint of
                    % the obstacle. to model that, we introduce binary variables to model
                    % violation of the half-spaces, i.e.:
                    % dk(j)=true => j-th half-space is violated
                    
                    % for the agent not to collide with the obstacle, their
                    % respective x- and y-coordinates have to differ by at
                    % least these quantities:
                    required_gap = 0.5*(osize+asize)+MinSeparation;
                    actual_gap = y-opos;
                    % note that Obstacles.Size and Geometry.Size both
                    % denote width/height. however, the gap is w.r.t. to
                    % the origin, hence it must be bigger that the half
                    % the width and height.
                    
                    % [d=1] => [f(x)<=0] is modeled as f(x)<=M*(1-d)
                    dk = binvar(2*ydim, 1);
                    didx = 0;
                    for id = 1:ydim
                        didx = didx+1;
                        cons = cons + [ implies(dk(didx), actual_gap(id)>=required_gap(id)) ];
                        didx = didx+1;
                        cons = cons + [ implies(dk(didx), actual_gap(id)<=-required_gap(id)) ];
                    end
                    % at least one half-space must be violated if the
                    % obstacle is visible (if .Visible=0, we don't impose
                    % collision avoidance at this time step)
                    
                    % TODO: figure out which formulation is faster
                    dmax = length(dk);
                    % dmax = length(dk)/2; % at most 2 can be violated in a
                    % 2D space
                    cons = cons + [ovisible<=sum(dk)<=dmax];
                    
                    % must bound all variables
                    allvars = [opos; osize; ovisible; actual_gap; required_gap; y; asize];
                    if isa(allvars, 'sdpvar')
                        cons = cons + [ -BigBound <= allvars(:) <= BigBound ];
                    end
                end
            end
            
        end

    end
end
