%% evil moving obstacle harrases a peaceful trajectory-following agent

clear
yalmip clear
close all

%% set up the playground
N = 15;     % prediction horizon
Ts = 0.25;  % sampling time
agent = moantool.LinearAgent.demo2D('PredictionHorizon', N, 'SamplingTime', Ts);
agent.Size.Value = [1; 1]; % agents width (in the x-axis) and length
% parametric position reference
agent.Y.Reference = 'parameter';
% one obstacle
obstacle = moantool.Obstacle(agent);
% fixed size
obstacle.Size.Value = [3; 3];
% the obstacle is always active
obstacle.Visible.Value = 1;
% but the obstacle will be free to move
obstacle.Position.Value = 'parameter';
% the planner optimizes agent's motion
minsep = [0.5; 0.5]; % minimal separation in the x- and y-axis
planner = moantool.Planner(agent, obstacle, 'MinSeparation', minsep, 'solver', 'gurobi');

%% closed-loop simulation
% create the simulator
psim = moantool.Simulator(planner);
% simulation parameters
x0 = [0; 0; 0; 0]; % initial point
Nsim = 200; % number of simulation steps
% obstacle follows a circular trajectory
obstpos = psim.circularTrajectory(Nsim, 'Radius', 4, 'Loops', 4);
psim.Parameters.Obstacles.Position.Value = obstpos;
% the agent should move to [4; 0] and stay there, moving only if necessary
% to avoid the obstacle
yref = psim.pointwiseTrajectory(Nsim, [4 0; 0 -4; -4 0; 0 4; 4 0]');
psim.Parameters.Agent.Y.Reference = yref;
% run the simulation
psim.run(x0, Nsim);
% plot the results (only show 10 predictions equaly distributed on 1:N)
psim.plot('Constraints', false, 'axis', [-10 10 -10 10], ...
    'Reference', false, 'Trail', true, ...
    'predictions', true, 'PredSteps', 10);

