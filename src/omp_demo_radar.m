%% agent follows a circular trajectory and avoids static obstacles detected by radar

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
    if MixedInteger == true % if MIQP, all obstacles are visible to the agent based on radar
        obstacles(i).Visible.Value = 'parameter';
    else % if time-varying constraints, all obstacles are not visible to the agent
        obstacles(i).Visible.Value = 0;
    end
    % all have fixed size
    obstacles(i).Size.Value = [3; 2];
    % all have floating position
    obstacles(i).Position.Value = 'parameter';
end
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

%% set the radar

% -1 value indicates that visibility of the obstacle will be determined by
% the radar
% TODO: drop this requirement
if MixedInteger == true
    for i = 1:length(obstacles)
        psim.Parameters.Obstacles(i).Visible.Value = -1;
    end
end

% radar detector: returns true if the obstacle is in the radar's range
RadarRadius = 5;
radar_detector = @(apos, opos, osize) psim.circularRadar(RadarRadius,...
    apos, opos, osize);

%% run the simulation
tic;
psim.run(x0, Nsim, 'RadarDetector', radar_detector)
simtime = toc

%% Value of error
trackQual = 0;
for k = 1:Nsim
    trackQual = trackQual + (psim.Results.Y(:,k) - yref(:,k))'*eye(ny)*(psim.Results.Y(:,k)...
        - yref(:,k));
end
trackQual

%% plot the results
% save figures: paramter - 'SaveFigs',[step1,step2,step3,step4]
% pause(6)
% radar plotter plots the radar range w.r.t. current position of the agent
radar_plotter = @(apos) viscircles(apos', RadarRadius);
if MixedInteger == true
    psim.plot('RadarDetector', radar_detector,...
        'RadarPlotter', radar_plotter, 'axis', [-15 15 -15 15],...
        'Reference', true, 'trail', true, 'predictions', true,...
        'predsteps', 10, 'delay', 0.1, 'textSize', 24,...
        'textFont', 'CMU Serif','SaveFigs',[138,168,220]);
else
    psim.plot('RadarDetector', radar_detector,...
        'RadarPlotter', radar_plotter, 'axis', [-15 15 -15 15],...
        'Reference', true, 'trail', true, 'predictions', true,...
        'predsteps', 10, 'delay', 0.1, 'textSize', 24,...
        'textFont', 'CMU Serif', 'Constraints', true,'SaveFigs',[138,168,220]);
end
