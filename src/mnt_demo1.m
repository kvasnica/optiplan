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
agent.Y.Reference = [0; 0]; % track the origin
obstacles = [];
planner = moantool.Planner(agent, obstacles, 'solver', 'gurobi');

% that's it! from now on, it's just about the implementation

%% single optimization 
% to obtain optimal control inputs for a particular state, run the
% planner.optimize() method:
x = [0; 10; 0; -10];
[u, problem, openloop] = planner.optimize(x);
% non-zero value of "problem" indicates, well, a problem :)
if problem~=0
    error(yalmiperror(problem));
end

%% closed-loop simulation based on repetitive optimizations
% the planner is implemented in a receding horizon fashion where the
% optimization is repeated at each discrete time step for current values of
% system's states

x0 = [0; 10; 0; -10]; % initial point
Nsim = 60; % number of simulation steps

% load sample dynamics of the plant
simdata = moantool.LinearAgent.demo2Ddata(Ts);

% storage for closed-loop profiles of states, inputs and outputs
X = x0;
U = [];
Y = [];
% simulation loop
for k = 1:Nsim
    x = X(:, end); % last known state
    [u, prob] = planner.optimize(x);
    if prob~=0
        error('Problem at step %d: %s', k, yalmiperror(prob));
    end
    % state update
    xn = simdata.A*x+simdata.B*u;
    y = simdata.C*x;
    X = [X, xn];
    Y = [Y, y];
    U = [U, u];
end

% plot the profiles in the x-y plane
plot(Y(1, :), Y(2, :), 'linewidth', 2);
xlabel('x position'); ylabel('y position')
% plot the control objective (reaching the origin)
hold on
plot(0, 0, 'rx', 'markersize', 20);
