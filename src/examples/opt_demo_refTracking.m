%% agent follows a circular trajectory

clear
yalmip clear
close all
clc
addpath(genpath('optiplan'))
warning off

%% set up the playground
N = 30;     % prediction horizon
Ts = 0.25;  % sampling time
agent = optiplan.LinearAgent.demo2D('PredictionHorizon', N, 'SamplingTime', Ts);
% position reference will be time-varying
agent.Y.Reference = 'parameter';
% agents width (in the x-axis) and length (in the y-axis)
agent.Size.Value = [1; 1];
% no obstacles
obstacles = [];
% the planner optimizes agent's motion
planner = optiplan.Planner(agent, obstacles, 'solver', 'gurobi');

%% closed-loop simulation
% create the simulator
psim = optiplan.Simulator(planner);
% simulation parameters
x0 = [0; 0; 0; 0]; % initial point
Nsim = 350; % number of simulation steps
% use a circular reference
yref = psim.circularTrajectory(Nsim, 'Radius', 10, 'Loops', 2);
psim.Parameters.Agent.Y.Reference = yref;
% run the simulation
psim.run(x0, Nsim);
% plot the results
psim.plot('axis', [-15 15 -15 15], 'Reference', true, 'predictions', true,...
    'predsteps', 10, 'trail', true, 'textSize', 24, 'textFont', 'CMU Serif');
