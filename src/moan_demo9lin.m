% Trajectory following with a nonlinear dynamics using intermediate
% linearization

clear
yalmip clear
close all

% RHS of the continous-time nonlinear ODE for state update
f = @(x, u) [x(3); x(4); u(1); u(1)*u(2)];
% RHS of the output equation
g = @(x, u) x(1:2);

% Note: the dynamics above is just made up to proove the concept. Ideally,
% we would like to have
%    f = @(x, u) [x(3); x(4); u(1)*sin(u(2)); u(1)*cos(u(2))];
% But for that one we don't get good results yet since we need a state
% estimator with disturbance modeling.

% Simple discretization via forward Euler:
Ts = 1; % sampling time
% Create an agent with nonlinear dynamics that will be linearized around
% some trajectory (will be updated automatically during the simulation)
N = 10; % prediction horizon
agent = moantool.LinearizedAgent('nx', 4, 'nu', 2, 'ny', 2, 'PredictionHorizon', N);
% Set nonlinear dynamics
agent.StateEq = @(x, u, ~) x+Ts*f(x, u);
agent.OutputEq = @(x, u, ~) g(x, u);
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

% Two fixed obstacles
obstacles = moantool.Obstacle(agent, 2);
obstacles(1).Position.Value = [0; -5];
obstacles(1).Size.Value = [1; 1];
obstacles(1).Visible.Value = 1;
obstacles(2).Position.Value = [0; 5];
obstacles(2).Size.Value = [1; 1];
obstacles(2).Visible.Value = 1;

% Create the planner
planner = moantool.Planner(agent, obstacles, 'solver', 'gurobi');

% Create the simulator
psimlin = moantool.Simulator(planner);

% initial linearization trajectory
Xlin = [-5 -5 -4.965 -4.918 -4.839 -4.755 -4.641 -4.525 -4.385 -4.217;0 0 -0.4988 -0.9379 -1.238 -1.56 -1.843 -2.114 -2.422 -2.684;0 0.03493 0.04657 0.07987 0.08315 0.1143 0.1165 0.1394 0.1687 0.1691;0 -0.4988 -0.4391 -0.3004 -0.3218 -0.2829 -0.2708 -0.3084 -0.2614 -0.268];
Ulin = [-0.5 0.0608 0.1426 -0.02164 0.04987 0.0123 -0.04402 0.05534 -0.00658 0.04709;-0.06992 0.1927 0.2357 -0.1523 0.6741 0.1782 -0.5471 0.5587 -0.06537 0.764];
psimlin.UserData.Xlin = Xlin(:, 1);
psimlin.UserData.Ulin = Ulin(:, 1);

% We want to follow a circular reference trajectory
Nsim = 100;
R = 5;
psimlin.Parameters.Agent.Y.Reference = psimlin.circularTrajectory(Nsim, ...
    'Radius', R, 'Loops', 1);

% Run the simulation with intermediate linearization around trajectory
x0 = [-R; 0; 0; 0];
psimlin.run(x0, Nsim)
psimlin.plot('Axis', [-10 10 -10 10], 'Reference', true, 'Trail', true, 'Predictions', false)
