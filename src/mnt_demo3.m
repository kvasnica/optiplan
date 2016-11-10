%% control of a planar vehicle:
% obstacles:
%   * multiple stationary obstacles
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
agent = moantool.LinearAgent.demo2D('PredictionHorizon', N, 'SamplingTime', Ts);
agent.Y.Reference = [-7; -1];

% the agent will be a rectangle of width=0.5 and height=0.5
agent_size = [0.5; 0.5];
agent.Size.Value = agent_size;

% we will add multiple obstacles:
obstacle_position = {};
obstacle_size = {};
% first obstacle
obstacle_position{end+1} = [4; -4];
obstacle_size{end+1} = [4; 2];
% second obstacle
obstacle_position{end+1} = [0; -6];
obstacle_size{end+1} = [2; 8];
% the third is a tricky one! it blocks the shortest path to the origin
obstacle_position{end+1} = [-2; -0.6];
obstacle_size{end+1} = [8; 1];

% number of obstacles
nObstacles = length(obstacle_position);
% create obstacles
obstacles = moantool.Obstacle(agent, nObstacles);
% set their parameters
for i = 1:nObstacles
    obstacles(i).Size.Value = obstacle_size{i};
    obstacles(i).Position.Value = obstacle_position{i};
    % the obstacle will be always visible to the agent
    obstacles(i).Visible.Value = 1;
end

%% get the optimization-based planner
planner = moantool.Planner(agent, obstacles, 'solver', 'gurobi');

%% closed-loop simulation

% create the simulator
psim = moantool.Simulator(planner);
% run the simulation
x0 = [0; 10; 0; -10]; % initial point
Nsim = 60; % number of simulation steps
psim.run(x0, Nsim);
% visualize results
psim.plot('Axis', [-11 11 -11 2], 'Trail', true, 'Predictions', true, 'Delay', 0.1);
