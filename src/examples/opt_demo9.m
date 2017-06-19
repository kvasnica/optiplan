% Trajectory following with a nonlinear dynamics

% Point mass with acceleration circle constraint
% model per pages 33 and 34 of
% http://web.mit.edu/mobility/publications/Peters_PhD_Thesis.pdf
%
% Nonlinear dynamics:
%   \ddot{X} = u1*sin(u2)
%   \ddot{Y} = u1*cos(u2)
%
% State vector:
%       [  X ] x-position   
%       [  Y ] y-position
%   z = [ vx ] x-speed
%       [ vy ] y-speed
%
% Input vector:
%       [ u1 ] acceleration
%   u = [ u2 ] angle
%
% Overall dynamics:
%   \dot{z} = f(z, u) = [ \dot{X}, \dot{Y}, u1*sin(u2), u1*cos(u2) ]^T
%                     = [ z(3), z(4), u(1)*sin(u(2)), u(1)*cos(u(2)) ]'

clear
yalmip clear
close all


% RHS of the ODE for state update
f = @(x, u) [x(3); x(4); u(1)*sin(u(2)); u(1)*cos(u(2))];
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
agent = optiplan.NonlinearAgent('nx', 4, 'nu', 2, 'ny', 2, 'PredictionHorizon', N);
% Set nonlinear dynamics
agent.StateEq = state_eq;
agent.OutputEq = output_eq;
% Fixed size of the agent
agent.Size.Value = [1; 1]; % width, height
% Input constraints
agent.U.Min = [-0.5; -pi];   % +/- 0.5 acceleration bound
agent.U.Max = [0.5; pi];     % +/- pi bound on the angle
% Slightly penalize control inputs to keep them small
agent.U.Penalty = 1e-3*eye(agent.nu);
agent.U.Reference = zeros(agent.nu, 1);

% ------- this will be a typical setting for many cases:
% Fixed penalty on position tracking
agent.Y.Penalty = eye(agent.ny);
% The position reference can be time-varying
agent.Y.Reference = 'parameter';
% No position constraints
agent.Y.Min = -Inf(agent.ny, 1);
agent.Y.Max = Inf(agent.ny, 1);
% No state constraints
agent.X.Min = -Inf(agent.nx, 1);
agent.X.Max = Inf(agent.nx, 1);
% No state reference/penalty
agent.X.Penalty = zeros(agent.nx);
agent.X.Reference = zeros(agent.nx, 1); % can be anything
% -------

% No obstacles
obstacles = [];

% Create the planner
planner = optiplan.Planner(agent, obstacles, 'solver', 'fmincon');

% Create the simulator
psim = optiplan.Simulator(planner);

% We want to follow a circular reference trajectory
Nsim = 100;
R = 5;
psim.Parameters.Agent.Y.Reference = psim.circularTrajectory(Nsim, ...
    'Radius', R, 'Loops', 1);

% Run the simulation
x0 = [-R; 0; 0; 0];
psim.run(x0, Nsim)
psim.plot('Axis', [-10 10 -10 10], 'Reference', true, 'Predictions', true, 'trail', true)
