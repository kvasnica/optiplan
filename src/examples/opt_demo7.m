%% agent follows a circular trajectory and avoids static obstacles detected by a radar

clear
yalmip clear
close all

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
    % for radar-based detection, the visibility of the obstacle needs to be
    % a parameter (will be updated during simulation automatically based on
    % radar data)
    obstacles(i).Visible.Value = 'parameter';
    % all have fixed size
    obstacles(i).Size.Value = [1; 1];
end
% positions of respective obstacles:
obstacles(1).Position.Value = [0; 10];
obstacles(2).Position.Value = [10; 0];
obstacles(3).Position.Value = [0; -10];
obstacles(4).Position.Value = [-10; 0];
% the planner optimizes agent's motion
minsep = agent.Size.Value; % minimal separation gap between the agent and the obstacles
planner = optiplan.Planner(agent, obstacles, 'MinSeparation', minsep, 'solver', 'gurobi');

%% closed-loop simulation
% create the simulator
psim = optiplan.Simulator(planner);
% simulation parameters
x0 = [0; 0; 0; 0]; % initial point
Nsim = 125; % number of simulation steps
% use a circular reference
yref = psim.circularTrajectory(Nsim, 'Radius', 10, 'Loops', 1);
psim.Parameters.Agent.Y.Reference = yref;
% -1 value indicates that visibility of the obstacle will be determined by
% the radar
% TODO: drop this requirement
for i = 1:length(obstacles)
    psim.Parameters.Obstacles(i).Visible.Value = -1;
end

% radar detector: returns true if the obstacle is in the radar's range
RadarRadius = 7;
radar_detector = @(apos, opos, osize) psim.circularRadar(RadarRadius, apos, opos, osize);
% run the simulation
psim.run(x0, Nsim, 'RadarDetector', radar_detector);
%% plot the results
% radar plotter plots the radar range w.r.t. current position of the agent
radar_plotter = @(apos) viscircles(apos', RadarRadius);
% plot the results
psim.plot('RadarDetector', radar_detector, 'RadarPlotter', radar_plotter, ...
    'Constraints', false, 'axis', [-15 15 -15 15], 'Reference', true, ...
    'trail', true, 'predictions', true, 'predsteps', 10, 'delay', 0.1);
