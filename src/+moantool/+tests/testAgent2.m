function agent = testAgent2()
% no obstacles

import moantool.*
%% MPC setup without parameters
Ts = 0.25; A = [1 0; Ts 1]; B = [Ts; 0.5*Ts^2]; C = [0 1];
mpcoptions.A = [A, zeros(2); zeros(2) A];
mpcoptions.B = [B zeros(2, 1); zeros(2, 1) B];
mpcoptions.C = [C zeros(1, 2); zeros(1, 2) C];
mpcoptions.Qy = 10*eye(2); mpcoptions.Qu = eye(2);
mpcoptions.umin = [-2; -2]; % minimal accelerations in x/y axis
mpcoptions.umax = [2; 2]; % maximal accelerations in x/y axis
mpcoptions.ymin = [-19; -19]; % minimal positions in x/y axis
mpcoptions.ymax = [19; 19]; % maximal positions in x/y axis
vmin = [-2; -2]; % minimal speeds in x/y axis
vmax = [2; 2]; % maximal speeds in x/y axis
mpcoptions.xmin = [vmin(1); mpcoptions.ymin(1); vmin(2); mpcoptions.ymin(2)];
mpcoptions.xmax = [vmax(1); mpcoptions.ymax(1); vmax(2); mpcoptions.ymax(2)];

%% agent setup
N = 10;
agent = LinearAgent('nx', 4, 'nu', 2, 'ny', 2, 'PredictionHorizon', N);
% constant dynamics
agent.A.Value = mpcoptions.A;
agent.B.Value = mpcoptions.B;
agent.C.Value = mpcoptions.C;
agent.D.Value = zeros(2, 2);
agent.f.Value = zeros(4, 1);
agent.g.Value = zeros(2, 1);
assert(isequal(agent.A.Value, mpcoptions.A));
assert(isequal(agent.B.Value, mpcoptions.B));
assert(isequal(agent.C.Value, mpcoptions.C));

% constant geometry
agent.Size.Value = [1; 1];

% constant constraints
agent.X.Min = mpcoptions.xmin;
agent.X.Max = mpcoptions.xmax;
agent.Y.Min = mpcoptions.ymin;
agent.Y.Max = mpcoptions.ymax;
agent.U.Min = mpcoptions.umin;
agent.U.Max = mpcoptions.umax;

% no reference/penalty on states
agent.X.Reference = zeros(agent.nx, 1);
agent.X.Penalty = zeros(agent.nx);
% constant zero reference on inputs
agent.U.Reference = zeros(agent.nu, 1);
% constant penalty on inputs
agent.U.Penalty = mpcoptions.Qu;
% constant penalty on tracking of output references
agent.Y.Penalty = mpcoptions.Qy;
% constant reference on outputs
agent.Y.Reference = 30*ones(agent.ny, 1);

end
