%% agent follows a circular trajectory and avoids static obstacles

clear
yalmip clear
close all
clc
addpath(genpath('optiplan'))
warning off

%% set up the playground
N = 30;     % prediction horizon
Ts = 0.25;  % sampling time
nx = 4; nu = 2; ny = 2;
agent = optiplan.LinearAgent.demo2D('PredictionHorizon', N, 'SamplingTime', Ts);
agent.Size.Value = [1; 1]; % agents width (in the x-axis) and length

% Decide between Mixed Integer and Constraint Change approach
% MixedInteger = true;
MixedInteger = false;

% for time-varying constraints, these must be sat as parameters
if MixedInteger == false
    agent.Y.Min = 'parameter';
    agent.Y.Max = 'parameter';
end
% position reference will be time-varying
agent.Y.Reference = 'parameter';
% 4 obstacles
obstacles = optiplan.Obstacle(agent, 4);
for i = 1:length(obstacles)
    if MixedInteger == true % if MIQP, all obstacles are visible to the agent
        obstacles(i).Visible.Value = 1;
    else % if time-varying constraints, all obstacles are not visible to the agent
        obstacles(i).Visible.Value = 0;
    end
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
planner = optiplan.Planner(agent, obstacles, 'MinSeparation', minsep,...
    'solver', 'gurobi', 'MixedInteger', MixedInteger);

%% closed-loop simulation
% create the simulator
psim = optiplan.Simulator(planner);
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

%% Value of error
trackQual = 0;
for k = 1:Nsim
    trackQual = trackQual + (psim.Results.Y(:,k) - yref(:,k))'*eye(ny)*(psim.Results.Y(:,k) - yref(:,k));
end
trackQual

%% plot the results
% save figures: paramter - 'SaveFigs',[step1,step2,step3,step4]
% pause(6)
if MixedInteger == true
    psim.plot('axis', [-15 15 -15 15], 'Reference', true, 'trail', true,...
        'predictions', true, 'predsteps', 10, 'delay', 0.1,...
        'textSize', 24,'textFont', 'CMU Serif');
else
    psim.plot('axis', [-15 15 -15 15], 'Reference', true, 'trail', true,...
        'predictions', true, 'predsteps', 10, 'delay', 0.1,...
        'textSize', 24, 'textFont', 'CMU Serif', 'Constraints', true);
end
