function [P, results] = moan_demo8
% Trajectory planning with obstacle avoidance and nonlinear constraints

% Notes:
%  * prediction horizon must be fixed
%  * starting location can be arbitrary
%  * final location is fixed (but can be made parametric, contact me)
%  * upper speed bound is fixed (but can be made parametric)
%  * minimal separation gap is fixed

% We want to generate a trajectory from a given starting point which meets
% following criteria:
%   * the planar motion of the agent is driven by horizontal and vertical
%     speeds vx and vy, respectively
%   * the overal speed, i.e., v = sqrt(vx^2+vy^2) is bounded
%   * at the end of the prediction horizon the agent's position must be as
%     close as possible to the desired location

% We model agent's dynamics as two decoupled integrators, i.e.,
%   \dot{p}_x = v_x
%   \dot{p}_y = v_y
% where p_x, p_y are the positions in the x- and y-axis, respectively. The
% control inputs are the horizontal (v_x) and vertical speeds (v_y). Since
% moantool only supports discrete-time formulations, we will discretize the
% dynamics using simple forward Euler rule with a given sampling time,
% i.e.:
%   p_x(k+1) = p_x(k) + Ts*v_x(k)
%   p_y(k+1) = p_y(k) + Ts*v_y(k)
% This will be subsequently translated to
%     x(k+1) = A*x(k) + B*u(k)
% with x(k) = [p_x(k); p_y(k)] and u(k) = [v_x(k); v_y(k)].

Ts = 1; % sampling time
A = eye(2); B = Ts*eye(2);  % x(k+1) = A*x(k) + B*u(k)
nx = 2; nu = 2; ny = 2;
% The horizon must be long enough for the agent to be able to reach the
% final location despite constraints on the overal speed
N = 30; 
agent = moantool.LinearAgent('Nx', 2, 'Nu', 2, 'Ny', 2, 'PredictionHorizon', N);
agent.Size.Value = [0.5; 0.5]; % width and height of the agent

% The agent's linear dynamics is time-invariant
agent.A.Value = A;
agent.B.Value = B;
agent.f.Value = zeros(nx, 1);
agent.C.Value = eye(ny); % in moantool, the outputs must always be the positions
agent.D.Value = zeros(ny, nu);
agent.g.Value = zeros(ny, 1);

% No constraints on positions, and speeds in individual axes (only on the
% overall speed, that one will be added later):
agent.X.Min = -Inf(nx, 1);
agent.X.Max = Inf(nx, 1);
agent.U.Min = -Inf(nu, 1);
agent.U.Max = Inf(nu, 1);
agent.Y.Min = -Inf(ny, 1);
agent.Y.Max = Inf(ny, 1);

% No penalties/references on states/inputs
agent.X.Penalty = zeros(nx);
agent.X.Reference = zeros(nx, 1);
agent.U.Penalty = zeros(nu);
agent.U.Reference = zeros(nu, 1);

% The output (position) reference can be time-varying, as well as the
% position penalty
agent.Y.Penalty = 'parameter';
agent.Y.Reference = 'parameter';

% But here's the trick: we want to limit the overal speed, given by
%  v = sqrt(vx^2+vy^2). In our notation that means that
%    sqrt(u(1, k)^2 + u(2, k)^2) <= vmax for k = 1,...,N (remember
% that Matlab indexes from one, hence u(1, k) = v_x(k-1)).
%
% To make the problem a bit simpler to solve, we square both sides of the
% inequality to get
%    (u(1, k)^2 + u(2, k)^2) <= vmax^2
% But it's still a nonlinear constraint (albeit convex).
%
% Nonlinear constraints are added via a custom function handle. It must
% point to a function which takes 4 inputs:
%  * the state predictions X = [x_0, \ldots, x_{N}]
%  * the input predictions U = [u_0, \ldots, u_{N-1}]
%  * the output predictions Y = [y_0, \ldots, y_{N-1}]
%  * the agent object
%
% Additionaly, we want the agent to be stationary at the end of the
% optimized path. I.e., u_{N-1}=0.
%
% Both the position constaints as well as the bound on absolute speed will
% be added via a function:
vmax = 3;
agent.ConstraintsFun = @(X, U, Y, agent) myConstraints(U, agent, vmax);

% In addition, we want to minimize the length of the path. Since we do
% point-wise optimization, this can be done by minimizing
%   \sum_{k=0}^{N-1} (y_k - y_{k-1})'*(y_k - y_{k-1})
% where the terms in the sum are squared distances between two consecutive
% points on the path.
%
% Custom objectives like this one can be added via the ObjectiveFun
% property of the agent. It also has to be a function handle with 4 inputs:
agent.ObjectiveFun = @(X, U, Y, agent) myObjective(Y, agent);

% We also consider three stationary obstacles:
nObstacles = 3;
obstacles = moantool.Obstacle(agent, nObstacles);
obstacles(1).Position.Value = [10; -4]; % center of the obstacle
obstacles(1).Size.Value = [8; 4]; % width and height
obstacles(1).Visible.Value = 1; % it's visible to the agent
obstacles(2).Position.Value = [-0.5; -8];
obstacles(2).Size.Value = [5; 12];
obstacles(2).Visible.Value = 1;
obstacles(3).Position.Value = [0; -0.6];
obstacles(3).Size.Value = [8; 1];
obstacles(3).Visible.Value = 1;

% Now we create a planner which will optimize the path. Due to the
% nonlinear speed constraints, the problem will be solved as a
% mixed-integer quadratically constrained quadratic program (MI-QCQP).
% Fortunately for us, the nonlinear constraints are at least convex.

% In addition, we want the agent to keep at least one multiple of its size
% as a safety separation gap between itself and the obstacles
minsep = 1*agent.Size.Value;
planner = moantool.Planner(agent, obstacles, 'solver', 'gurobi', 'MinSeparation', minsep);

% Finally we can optimize the path starting from a given point:
start = [10; -10];
destination = [-5; -2];

% Fix the planner's parameters. Here, we only heavily penalize the
% deviation of the final predicted position from the desired point and
% don't care about the intermediate points:
planner.Parameters.Agent.Y.Penalty = [repmat(zeros(ny), 1, N-1), 1e4*eye(ny)];
planner.Parameters.Agent.Y.Reference = destination;

% Run the optimization
[~, problem, results] = planner.optimize(start);

% Any issues?
if problem~=0
    error(yalmiperror(problem));
end
% the optimized path is in results.Y

close all
figure; hold on; grid on
% Plot the obstacles
for i = 1:length(obstacles)
    moantool.utils.plotRectangle('Position', obstacles(i).Position.Value, ...
        'Size', obstacles(i).Size.Value);
end
% Plot the optimize trajectory
plot(results.Y(1, :), results.Y(2, :), 'marker', 'x');
% Plot the starting and finishing point
plot(start(1), start(2), 'rs', destination(1), destination(2), 'gs', 'markersize', 18);

% Return the path
P = results.Y;

end

function cons = myConstraints(U, agent, vmax)
% Custom constraints:
%  * final speed must be zero
%  * absolute speed must be upper-bounded by "vmax"

cons = [];

% At the end of the path the agent has to be stationary
cons = cons + [ U(:, end)==0 ];

% Now add nonlinear input constraints for each step of the prediction
% window
for k = 1:agent.N
    % Absolute speed at the k-th step of the prediction window
    vk = U(:, k)'*U(:, k); % squared 2-norm of U(:, k)
    % Bound the speed from above, remember that we have squared the 2-norm
    cons = cons + [ vk <= vmax^2 ];
end

end

function J = myObjective(Y, agent)
% Custom objective function

% Minimize the length of the path by minimizing the distances of every two
% consecutive points on the path:
%   \sum_{k=1}^{N} (y_k - y_{k-1})'*(y_k - y_{k-1})
%
% Note: try to keep the objective function quadratic and convex whenever
% possible.
J = 0;
for k = 2:agent.N
    dY = Y(:, k)-Y(:, k-1);
    J = J + dY'*dY;
end

end
