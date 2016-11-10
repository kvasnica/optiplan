function t_Simulator()

if exist('functiontests')
    tests = functiontests(localfunctions);
    run(tests);
else
    test_constructor1();
    test_constructor2();
    test_listParameters();
    test_listMissing();
    test_run1();
    test_run2();
    test_run3;
    test_run4;
end

end

function testInput = setupOnce(testCase)
% N = 10;
% dim = 2;
% agent = moantool.LinearAgent('nx', 4, 'nu', 2, 'ny', dim, 'PredictionHorizon', N);
% testCase.TestData.N = N;
% testCase.TestData.dim = dim;
% testCase.TestData.agent = agent;
% testInput.TestData = testCase.TestData;
end

function test_constructor1(testInput)
% input must be a Planner object
msg = moantool.utils.run_in_caller(@() moantool.Simulator([]));
moantool.utils.assert_errmsg(msg, 'The first input must be a Planner object');
end

function test_constructor2(testInput)
N = 10; Ts = 0.25;
agent = moantool.LinearAgent.demo2D('PredictionHorizon', N, 'SamplingTime', Ts);
agent.Y.Reference = 'parameter';
obstacles = [];
planner = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
psim = moantool.Simulator(planner);
assert(isempty(psim.Results.X));
assert(isempty(psim.Results.U));
assert(isempty(psim.Results.Y));
assert(isempty(psim.Results.Predictions));
assert(isstruct(psim.Parameters));
assert(~isempty(psim.Parameters.Agent.Y.Reference));
end

function test_listParameters(testInput)
N = 10; Ts = 0.25;
agent = moantool.LinearAgent.demo2D('PredictionHorizon', N, 'SamplingTime', Ts);
agent.Y.Reference = 'parameter';
obstacles = [];
planner = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
psim = moantool.Simulator(planner);
T=evalc('psim.listParameters');
assert(length(T)==43);
end

function test_listMissing(testInput)
N = 10; Ts = 0.25;
agent = moantool.LinearAgent.demo2D('PredictionHorizon', N, 'SamplingTime', Ts);
agent.Y.Reference = 'parameter';
obstacles = [];
planner = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
psim = moantool.Simulator(planner);
T=evalc('psim.listMissing');
assert(length(T)==44);
% now set the value
psim.Parameters.Agent.Y.Reference = [1; 1];
T=evalc('psim.listMissing');
assert(isempty(T)); % nothing must be missing
end

function test_run1(testInput)
N = 10; Ts = 0.25;
agent = moantool.LinearAgent.demo2D('PredictionHorizon', N, 'SamplingTime', Ts);
agent.Y.Reference = 'parameter';
obstacles = [];
planner = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
psim = moantool.Simulator(planner);
% x0 must be correct
msg = moantool.utils.run_in_caller(@() psim.run([], 1));
moantool.utils.assert_errmsg(msg, 'x0 must be a 4x1 vector.');
% parameters must be set
assert(~isempty(psim.listMissing));
msg = moantool.utils.run_in_caller(@() psim.run(zeros(4, 1), 1));
moantool.utils.assert_errmsg(msg, 'There are missing values.');
end

function test_run2(testInput)
N = 10; Ts = 0.25;
agent = moantool.LinearAgent.demo2D('PredictionHorizon', N, 'SamplingTime', Ts);
obstacles = [];
planner = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
psim = moantool.Simulator(planner);
x0 = [0; 10; 0; -10]; Nsim = 60;
psim.run(x0, Nsim);
assert(isequal(size(psim.Results.X), [agent.nx, Nsim+1]));
assert(isequal(size(psim.Results.U), [agent.nu, Nsim]));
assert(isequal(size(psim.Results.Y), [agent.ny, Nsim]));
Uexp = [-2 -2 -2 -2 -1.4521e-10 7.162e-12 4.8755e-11 -3.9791e-12 1.3537e-11 -1.6822e-11 -3.6654e-13 -1.3827e-11 2.5262e-11 1.6343e-11 -1.0307e-10 -4.8843e-12 8.587e-09 0.31093 0.63289 0.83178 0.93123 0.9536 0.9191 0.84538 0.74725 0.6367 0.52305 0.4132 0.31193 0.22223 0.14565 0.082614 0.032705 -0.0050726 -0.032084 -0.049887 -0.060094 -0.064273 -0.063878 -0.0602 -0.054346 -0.047233 -0.039588 -0.031965 -0.024762 -0.018246 -0.012571 -0.0078049 -0.0039489 -0.00095454 0.0012585 0.0027898 0.0037475 0.00424 0.0043706 0.0042327 0.0039083 0.0034666 0.0029634 0.0024427;2 2 2 2 1.4521e-10 -7.162e-12 -4.8755e-11 3.979e-12 -1.3537e-11 1.6822e-11 3.6637e-13 1.3828e-11 -2.5262e-11 -1.6343e-11 1.0308e-10 4.9409e-12 -1.8083e-09 -0.31093 -0.63289 -0.83178 -0.93123 -0.9536 -0.9191 -0.84538 -0.74725 -0.6367 -0.52305 -0.4132 -0.31193 -0.22223 -0.14565 -0.082614 -0.032705 0.0050726 0.032084 0.049887 0.060094 0.064273 0.063878 0.0602 0.054346 0.047233 0.039588 0.031965 0.024762 0.018246 0.012571 0.0078049 0.0039489 0.00095454 -0.0012585 -0.0027898 -0.0037475 -0.00424 -0.0043706 -0.0042327 -0.0039083 -0.0034666 -0.0029634 -0.0024427];
assert(norm(psim.Results.U-Uexp, Inf)<1e-4);
assert(isstruct(psim.Results.Predictions));
assert(length(psim.Results.Predictions)==Nsim);
end

function test_run3(testInput)
% test_run2 with parametric dynamics
N = 10; Ts = 0.25;
agent = moantool.LinearAgent.demo2D('PredictionHorizon', N, 'SamplingTime', Ts);
A = agent.A.Value;
B = agent.B.Value;
xmax = agent.X.Max;
agent.A.Value = 'parameter';
agent.B.Value = 'parameter';
agent.X.Max = 'parameter';
obstacles = [];
planner = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
psim = moantool.Simulator(planner);
x0 = [0; 10; 0; -10]; Nsim = 60;

% parameters must be set
T = evalc('psim.listMissing');
assert(length(T)==120);
msg = moantool.utils.run_in_caller(@() psim.run(x0, Nsim));
moantool.utils.assert_errmsg(msg, 'There are missing values.');

% all parameters set
x0 = [0; 10; 0; -10]; Nsim = 60;
psim.Parameters.Agent.A.Value = A;
psim.Parameters.Agent.B.Value = B;
psim.Parameters.Agent.X.Max = xmax;
psim.run(x0, Nsim, 'Preview', false);
assert(isequal(size(psim.Results.X), [agent.nx, Nsim+1]));
assert(isequal(size(psim.Results.U), [agent.nu, Nsim]));
assert(isequal(size(psim.Results.Y), [agent.ny, Nsim]));
Uexp = [-2 -2 -2 -2 -1.4521e-10 7.162e-12 4.8755e-11 -3.9791e-12 1.3537e-11 -1.6822e-11 -3.6654e-13 -1.3827e-11 2.5262e-11 1.6343e-11 -1.0307e-10 -4.8843e-12 8.587e-09 0.31093 0.63289 0.83178 0.93123 0.9536 0.9191 0.84538 0.74725 0.6367 0.52305 0.4132 0.31193 0.22223 0.14565 0.082614 0.032705 -0.0050726 -0.032084 -0.049887 -0.060094 -0.064273 -0.063878 -0.0602 -0.054346 -0.047233 -0.039588 -0.031965 -0.024762 -0.018246 -0.012571 -0.0078049 -0.0039489 -0.00095454 0.0012585 0.0027898 0.0037475 0.00424 0.0043706 0.0042327 0.0039083 0.0034666 0.0029634 0.0024427;2 2 2 2 1.4521e-10 -7.162e-12 -4.8755e-11 3.979e-12 -1.3537e-11 1.6822e-11 3.6637e-13 1.3828e-11 -2.5262e-11 -1.6343e-11 1.0308e-10 4.9409e-12 -1.8083e-09 -0.31093 -0.63289 -0.83178 -0.93123 -0.9536 -0.9191 -0.84538 -0.74725 -0.6367 -0.52305 -0.4132 -0.31193 -0.22223 -0.14565 -0.082614 -0.032705 0.0050726 0.032084 0.049887 0.060094 0.064273 0.063878 0.0602 0.054346 0.047233 0.039588 0.031965 0.024762 0.018246 0.012571 0.0078049 0.0039489 0.00095454 -0.0012585 -0.0027898 -0.0037475 -0.00424 -0.0043706 -0.0042327 -0.0039083 -0.0034666 -0.0029634 -0.0024427];
assert(norm(psim.Results.U-Uexp, Inf)<1e-4);
assert(isstruct(psim.Results.Predictions));
assert(length(psim.Results.Predictions)==Nsim);

psim.run(x0, Nsim, 'Preview', true);
assert(isequal(size(psim.Results.X), [agent.nx, Nsim+1]));
assert(isequal(size(psim.Results.U), [agent.nu, Nsim]));
assert(isequal(size(psim.Results.Y), [agent.ny, Nsim]));
Uexp = [-2 -2 -2 -2 -1.4521e-10 7.162e-12 4.8755e-11 -3.9791e-12 1.3537e-11 -1.6822e-11 -3.6654e-13 -1.3827e-11 2.5262e-11 1.6343e-11 -1.0307e-10 -4.8843e-12 8.587e-09 0.31093 0.63289 0.83178 0.93123 0.9536 0.9191 0.84538 0.74725 0.6367 0.52305 0.4132 0.31193 0.22223 0.14565 0.082614 0.032705 -0.0050726 -0.032084 -0.049887 -0.060094 -0.064273 -0.063878 -0.0602 -0.054346 -0.047233 -0.039588 -0.031965 -0.024762 -0.018246 -0.012571 -0.0078049 -0.0039489 -0.00095454 0.0012585 0.0027898 0.0037475 0.00424 0.0043706 0.0042327 0.0039083 0.0034666 0.0029634 0.0024427;2 2 2 2 1.4521e-10 -7.162e-12 -4.8755e-11 3.979e-12 -1.3537e-11 1.6822e-11 3.6637e-13 1.3828e-11 -2.5262e-11 -1.6343e-11 1.0308e-10 4.9409e-12 -1.8083e-09 -0.31093 -0.63289 -0.83178 -0.93123 -0.9536 -0.9191 -0.84538 -0.74725 -0.6367 -0.52305 -0.4132 -0.31193 -0.22223 -0.14565 -0.082614 -0.032705 0.0050726 0.032084 0.049887 0.060094 0.064273 0.063878 0.0602 0.054346 0.047233 0.039588 0.031965 0.024762 0.018246 0.012571 0.0078049 0.0039489 0.00095454 -0.0012585 -0.0027898 -0.0037475 -0.00424 -0.0043706 -0.0042327 -0.0039083 -0.0034666 -0.0029634 -0.0024427];
assert(norm(psim.Results.U-Uexp, Inf)<1e-4);
assert(isstruct(psim.Results.Predictions));
assert(length(psim.Results.Predictions)==Nsim);

end

function test_run4(testInput)
% omp_demo4 with time-varying output reference
N = 10;     % prediction horizon
Ts = 0.25;  % sampling time
agent = moantool.LinearAgent.demo2D('PredictionHorizon', N, 'SamplingTime', Ts);
agent.Y.Reference = 'parameter';
obstacles = [];
planner = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
x0 = [0; 10; 0; -10]; % initial point
Nsim = 80; % number of simulation steps
psim = moantool.Simulator(planner);
yref1 = [5; 5];
yref2 = [-5; -5];
psim.Parameters.Agent.Y.Reference = [repmat(yref1, 1, Nsim/2), repmat(yref2, 1, Nsim/2)];

% without preview
psim.run(x0, Nsim, 'Preview', false);
Yexp = [10 9.9375 9.75 9.4375 9.01538 8.53075 8.03075 7.53075 7.03965 6.57669 6.15862 5.79518 5.4905 5.24442 5.05373 4.91318 4.81629 4.75608 4.72555 4.71808 4.72765 4.74904 4.7778 4.81038 4.84397 4.87652 4.90658 4.93325 4.95606 4.97486 4.98977 5.00108 5.0092 5.01456 5.01767 5.01897 5.01891 5.01785 5.01615 5.01406 5.0118 4.94701 4.75722 4.44242 4.00378 3.50378 3.00378 2.50378 2.00378 1.50378 1.00378 0.503779 0.00377859 -0.496221 -0.996221 -1.49622 -1.99622 -2.49622 -2.98661 -3.44766 -3.86305 -4.22341 -4.52488 -4.76782 -4.95559 -5.09355 -5.1882 -5.24656 -5.27561 -5.28195 -5.27156 -5.24963 -5.22054 -5.18782 -5.15421 -5.12175 -5.09185 -5.06537 -5.04279 -5.0242;-10 -9.9375 -9.75 -9.4375 -9 -8.5 -8 -7.5 -7 -6.5 -6 -5.5 -5 -4.5 -4 -3.5 -3 -2.5 -2 -1.5 -1 -0.5 -5.91255e-11 0.5 1 1.5 2 2.5 2.99028 3.45107 3.86609 4.22601 4.52704 4.76954 4.9569 5.09449 5.18883 5.24693 5.27578 5.28196 5.27145 5.1915 4.98654 4.65659 4.21036 3.71036 3.21036 2.71036 2.21036 1.71036 1.21036 0.71036 0.21036 -0.28964 -0.78964 -1.28964 -1.78964 -2.28964 -2.78551 -3.26121 -3.69708 -4.08103 -4.40711 -4.67407 -4.88417 -5.04204 -5.15381 -5.22636 -5.26673 -5.28172 -5.27758 -5.25985 -5.23324 -5.20164 -5.16812 -5.13498 -5.10387 -5.0759 -5.05167 -5.03142];
assert(norm(psim.Results.Y-Yexp, Inf)<1e-3);

% with preview
psim.run(x0, Nsim, 'Preview', true);
Yexp = [10 9.9375 9.75 9.4375 9.01538 8.53075 8.03075 7.53075 7.03965 6.57669 6.15862 5.79518 5.4905 5.24442 5.05373 4.91318 4.81629 4.75608 4.72555 4.71808 4.72765 4.74904 4.7778 4.81038 4.84397 4.87652 4.90658 4.93325 4.95606 4.97486 4.98977 5.00108 4.97268 4.84454 4.59217 4.21579 3.74686 3.24686 2.74686 2.24686 1.74686 1.24686 0.746858 0.246858 -0.253142 -0.753142 -1.25314 -1.75314 -2.25314 -2.74998 -3.22827 -3.66775 -4.05588 -4.3863 -4.65751 -4.87155 -5.03294 -5.14774 -5.22279 -5.26516 -5.28168 -5.27864 -5.26165 -5.23549 -5.20409 -5.17057 -5.13731 -5.106 -5.07776 -5.05323 -5.0327 -5.01613 -4.9668 -4.82385 -4.55952 -4.17454 -3.70298 -3.20298 -2.70298 -2.20298;-10 -9.9375 -9.75 -9.4375 -9 -8.5 -8 -7.5 -7 -6.5 -6 -5.5 -5 -4.5 -4 -3.5 -3 -2.5 -2 -1.5 -1 -0.5 -5.91255e-11 0.5 1 1.5 2 2.5 2.99028 3.45107 3.86609 4.22601 4.49434 4.63766 4.65598 4.5493 4.31763 3.96095 3.50136 3.00136 2.50136 2.00136 1.50136 1.00136 0.501362 0.00136163 -0.498638 -0.998638 -1.49864 -1.99864 -2.49864 -2.98896 -3.44984 -3.865 -4.22508 -4.52626 -4.76892 -4.95643 -5.09415 -5.1886 -5.2468 -5.27572 -5.28196 -5.27149 -5.24951 -5.22039 -5.18765 -5.15405 -5.1216 -5.09171 -5.06525 -5.04268 -4.9876 -4.83942 -4.57017 -4.18145 -3.70861 -3.20861 -2.70861 -2.20861];
assert(norm(psim.Results.Y-Yexp, Inf)<1e-3);

end
