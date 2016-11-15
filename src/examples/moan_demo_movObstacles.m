%% agent follows a circular trajectory and avoids moving obstacles

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
% MixedInteger = true;
MixedInteger = false;

% position reference will be time-varying
agent.Y.Reference = 'parameter';
% 4 obstacles
obstacles = moantool.Obstacle(agent, 4);
for i = 1:length(obstacles)
    obstacles(i).Visible.Value = 1;
    % all have fixed size
    obstacles(i).Size.Value = [3; 2];
    % all have floating position
    obstacles(i).Position.Value = 'parameter';
end
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

% obstacles follow a circular trajectory
obstpos{1} = psim.circularTrajectory(Nsim, 'Radius', 10, 'Loops', 3,...
    'Center', [0;0], 'InitPoint', pi);
obstpos{2} = psim.circularTrajectory(Nsim, 'Radius', 10, 'Loops', 1,...
    'Center', [0;0], 'InitPoint', pi/2);
obstpos{3} = psim.circularTrajectory(Nsim, 'Radius', 2, 'Loops', 5,...
    'Center', [0;10]);
obstpos{4} = psim.circularTrajectory(Nsim, 'Radius', 3, 'Loops', 5,...
    'Center', [0;-10], 'InitPoint', pi/2);

for i = 1:length(obstacles)
    psim.Parameters.Obstacles(i).Position.Value = obstpos{i};
end

% use a circular reference
yref = psim.circularTrajectory(Nsim, 'Radius', 10, 'Loops', 2);
psim.Parameters.Agent.Y.Reference = yref;

%% run the simulation
tic;
psim.run(x0, Nsim)
simtime = toc

%% plot the results
% save figures: paramter - 'SaveFigs',[step1,step2,step3,step4]
pause(10)
if MixedInteger == true
    psim.plot('axis', [-15 15 -15 15], 'Reference', true, 'trail', true,...
        'predictions', true, 'predsteps', 10, 'delay', 0.1,...
        'textSize', 24,'textFont', 'CMU Serif');
else
    psim.plot('axis', [-15 15 -15 15], 'Reference', true, 'trail', true,...
        'predictions', true, 'predsteps', 10, 'delay', 0.1,...
        'textSize', 24, 'textFont', 'CMU Serif', 'Constraints', true);
end
