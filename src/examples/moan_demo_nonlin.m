% Trajectory following with a nonlinear dynamics
%
% Nonlinear dynamics:
%   \ddot{X} = (r/2)*cos(Theta)*(u(1)+u(2))
%   \ddot{Y} = (r/2)*sin(Theta)*(u(1)+u(2))
%   \ddot{Theta} = (r/L)*(u(2)-u(1))
%
% State vector:
%       [  X  ]   x-position   
%   z = [  Y  ]   y-position
%   	[ Theta ] difference between angular speeds of wheels
%
% Input vector:
%       [ u1 ] angular speed of left wheel
%   u = [ u2 ] angular speed of right wheel
%
% Constants
%   r   radius of wheels
%   L   distance between wheels
%
% Overall dynamics:
%   \dot{z} = f(z, u) = [ \dot{X}, \dot{Y}, \dot{Theta} ]^T
%                     = [ (r/2)*cos(Theta)*(u(1)+u(2)), (r/2)*sin(Theta)*(u(1)+u(2)), (r/L)*(u(2)-u(1)) ]'

clear
yalmip clear
close all
clc
warning off

%%
r = 0.03;
L = 0.1;
% RHS of the ODE for state update
f = @(x, u) [(r/2)*cos(x(3))*(u(1)+u(2)); (r/2)*sin(x(3))*(u(1)+u(2)); (r/L)*(u(2)-u(1))];
% RHS of the output equation
g = @(x, u) x(1:2);

% Simple discretization via forward Euler:
Ts = 1; % sampling time
% The state-update equation must take three inputs (current state, current
% input and the agent object) and return one output (successor state vector)
state_eq = @(x, u, ~) x+Ts*f(x, u);
% The output equation must take three inputs (current state, current
% input and the agent object) and return the x-y position vector
output_eq = @(x, u, ~) g(x, u);

% Create an agent with nonlinear dynamics
N = 10; % prediction horizon
nx = 3; nu = 2; ny = 2;
agent = moantool.NonlinearAgent('nx', nx, 'nu', nu, 'ny', ny, 'PredictionHorizon', N);
% Set nonlinear dynamics
agent.StateEq = state_eq;
agent.OutputEq = output_eq;
% Fixed size of the agent
agent.Size.Value = [1; 1]; % width, height
% Input constraints
agent.U.Min = [-40; -40];   % +/- 0.5 acceleration bound
agent.U.Max = [40; 40];     % +/- pi bound on the angle
% Slightly penalize control inputs to keep them small
agent.U.Penalty = 1e-3*eye(agent.nu);
agent.U.Reference = zeros(agent.nu, 1);

% Decide between Mixed Integer and Constraint Change approach
% MixedInteger = true;
MixedInteger = false;

% ------- this will be a typical setting for many cases:
% Fixed penalty on position tracking
agent.Y.Penalty = eye(agent.ny);
% The position reference can be time-varying
agent.Y.Reference = 'parameter';

% No state constraints
agent.X.Min = -Inf(agent.nx, 1);
agent.X.Max = Inf(agent.nx, 1);
% No state reference/penalty
agent.X.Penalty = zeros(agent.nx);
agent.X.Reference = zeros(agent.nx, 1); % can be anything
% -------

% % No obstacles
% obstacles = [];

obstacles = moantool.Obstacle(agent, 4);
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
obstacles(1).Position.Value = [0; -10];
obstacles(2).Position.Value = [10; 0];
obstacles(3).Position.Value = [0; 10];
obstacles(4).Position.Value = [-10; 0];

% Create the planner
minsep = agent.Size.Value; % minimal separation gap between the agent and the obstacles
planner = moantool.Planner(agent, obstacles, 'MinSeparation', minsep,...
    'solver', 'fmincon', 'MixedInteger', MixedInteger);

% Create the simulator
psim = moantool.Simulator(planner);

% We want to follow a circular reference trajectory
Nsim = 350;
R = 10;
yref = psim.circularTrajectory(Nsim, 'Radius', R, 'Loops', 2);
psim.Parameters.Agent.Y.Reference = yref;

x0 = [0; 0; 0];

%% Run the simulation

tic;
psim.run(x0, Nsim)
time = toc

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