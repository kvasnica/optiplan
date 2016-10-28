%% agent follows a circular trajectory and avoids static obstacles

clear
yalmip clear
close all
clc

%% set up the playground
N = 30;     % prediction horizon
Ts = 0.25;  % sampling time
agent = optiplan.LinearAgent.demo2D('PredictionHorizon', N, 'SamplingTime', Ts);
agent.Size.Value = [1; 1]; % agents width (in the x-axis) and length
% position reference will be time-varying
agent.Y.Reference = 'parameter';
% 4 obstacles
obstacles = optiplan.Obstacle(agent, 4);
for i = 1:length(obstacles)
    % all obstacles are visible to the agent
    obstacles(i).Visible.Value = 1;
    % all have fixed size
    obstacles(i).Size.Value = [2*i; i];
end
% positions of respective obstacles:
obstacles(1).Position.Value = [0; 10];
obstacles(2).Position.Value = [-10; 0];
obstacles(3).Position.Value = [0; -10];
obstacles(4).Position.Value = [10; 0];
% the planner optimizes agent's motion
minsep = agent.Size.Value; % minimal separation gap between the agent and the obstacles
planner = optiplan.Planner(agent, obstacles, 'MinSeparation', minsep, 'solver', 'gurobi');

%% closed-loop simulation
% create the simulator
psim = optiplan.Simulator(planner);
% simulation parameters
x0 = [0; 0; 0; 0]; % initial point
Nsim = 300; % number of simulation steps
% use a circular reference
yref = psim.circularTrajectory(Nsim, 'Radius', 10, 'Loops', 2);
psim.Parameters.Agent.Y.Reference = yref;
% run the simulation
psim.run(x0, Nsim);
%% plot the results
psim.plot('Constraints', false, 'axis', [-15 15 -15 15], 'Reference', true,...
    'trail', true, 'predictions', true, 'predsteps', 10, 'delay', 0.1,...
    'textSize', 24, 'textFont', 'CMU Serif');
