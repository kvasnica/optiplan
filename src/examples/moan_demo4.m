%% control of a planar vehicle:
% obstacles:
%   * none
% agent:
%   * zero size (i.e., a point)
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
agent.Size.Value = [1; 1]; % agents width (in the x-axis) and length
% position reference will be time-varying
agent.Y.Reference = 'parameter';

% no obstacles
obstacles = [];
planner = moantool.Planner(agent, obstacles, 'solver', 'gurobi');

%% closed-loop simulation

% simulation parameters
x0 = [0; 10; 0; -10]; % initial point
Nsim = 80; % number of simulation steps
% we will use a time-varying position reference here with yref1 for the
% first half and yref2 for the second half of the simulation
yref1 = [5; 5];
yref2 = [-5; -5];
% outlook of the reference over the whole simulation:
yref = [repmat(yref1, 1, Nsim/2), repmat(yref2, 1, Nsim/2)];

% create the simulator
psim = moantool.Simulator(planner);
% set the time-varying reference
psim.Parameters.Agent.Y.Reference = yref;
% run the simulation
psim.run(x0, Nsim, 'Preview', false);
% plot the results
psim.plot('Trail', true, 'Reference', false);
