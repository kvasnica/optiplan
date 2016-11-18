%% agent follows a circular trajectory and avoids static obstacles
% Time-Varying Constraints

clear
yalmip clear
close all
clc
addpath(genpath('moantool'))
warning off

%% set up the playground
N = 30;     % prediction horizon
Ts = 0.25;  % sampling time
nx = 4; nu = 2; ny = 2;
agent = moantool.LinearAgent.demo2D('PredictionHorizon', N, 'SamplingTime', Ts);
agent.Size.Value = [1; 1]; % agents width (in the x-axis) and length

% Decide between Mixed Integer and Constraint Change approach
MixedInteger = false;

% position reference will be time-varying
agent.Y.Reference = 'parameter';
% 4 obstacles
obstacles = moantool.Obstacle(agent, 4);
for i = 1:length(obstacles)
    obstacles(i).Visible.Value = 1;
    % all have fixed size
    obstacles(i).Size.Value = [3; 2];
end
% positions of respective obstacles:
obstacles(1).Position.Value = [0; 10];
obstacles(2).Position.Value = [10; 0];
obstacles(3).Position.Value = [0; -10];
obstacles(4).Position.Value = [-10; 0];
% the planner optimizes agent's motion
minsep = agent.Size.Value; % minimal separation gap between the agent and the obstacles
planner = moantool.Planner(agent, obstacles, 'MinSeparation', minsep,...
    'solver', 'gurobi', 'MixedInteger', MixedInteger);

%% closed-loop simulation
% create the simulator
psim = moantool.Simulator(planner);
% simulation parameters
x0 = [0; 0; 0; 0]; % initial point
Nsim = 350; % number of simulation steps
% use a circular reference
yref = psim.circularTrajectory(Nsim, 'Radius', 10, 'Loops', 2);
psim.Parameters.Agent.Y.Reference = yref;

%% run the simulation
tic;
psim.run(x0, Nsim)
simtime = toc

%% plot the results
psim.plot('Axis', [-15 15 -15 15], 'Reference', true, 'Trail', true,...
    'Predictions', true, 'PredSteps', 10, 'Delay', 0.1,...
    'textSize', 24, 'textFont', 'CMU Serif', 'Constraints', true);
