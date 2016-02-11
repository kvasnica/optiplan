%% control of a planar vehicle:
% obstacles:
%   * one stationary obstacle
% agent:
%   * non-zero size
%   * time-invariant dynamics
%   * time-invariant state/input/output constraints
%   * time-invariant input/output penalties
%   * tracking of a constant reference (the origin)

clear
yalmip clear
close all

%% set up the playground
N = 10;     % prediction horizon
Ts = 0.25;  % sampling time
agent = optiplan.LinearAgent.demo2D('PredictionHorizon', N, 'SamplingTime', Ts);
agent.Y.Reference = [0; 0]; % track the origin

% the agent will be a rectangle of width=0.5 and height=0.5
agent_size = [0.5; 0.5];
agent.Size.Value = agent_size;

% add a rectangular obstacle
obstacle = optiplan.Obstacle(agent);
% the obstacle will be a rectangle with width=4 and height=2
obstacle_size = [4; 2];
obstacle.Size.Value = obstacle_size;
% the obstacle will be stationary at x=4, y=-4
obstacle_position = [4; -4];
obstacle.Position.Value = obstacle_position;
% the obstacle will be always visible to the agent
obstacle.Visible.Value = 1;

% get the optimization-based planner
planner = optiplan.Planner(agent, obstacle, 'solver', 'gurobi');

%% closed-loop simulation based on repetitive optimizations
% the planner is implemented in a receding horizon fashion where the
% optimization is repeated at each discrete time step for current values of
% system's states

x0 = [0; 10; 0; -10]; % initial point
Nsim = 100; % number of simulation steps

% load sample dynamics of the plant
simdata = optiplan.LinearAgent.demo2Ddata(Ts);

% storage for closed-loop profiles of states, inputs and outputs
X = x0;
U = [];
Y = [];
% simulation loop
for k = 1:Nsim
    x = X(:, end); % last known state
    [u, prob] = planner.optimize(x);
    if prob~=0
        error('Problem at step %d: %s', k, yalmiperror(prob));
    end
    % state update
    xn = simdata.A*x+simdata.B*u;
    y = simdata.C*x;
    X = [X, xn];
    Y = [Y, y];
    U = [U, u];
end

% plot the obstacle
optiplan.utils.plotRectangle('Position', obstacle_position, ...
    'Size', obstacle_size, 'color', 'y');
hold on
% plot the profiles in the x-y plane
plot(Y(1, :), Y(2, :), 'linewidth', 2);
% plot the control objective (reaching the origin)
plot(0, 0, 'rx', 'markersize', 20);
xlabel('x position'); ylabel('y position')
grid on
axis([-11 11 -11 11])
