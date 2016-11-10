function t_Agent()
% tests for the Agent class

if exist('functiontests')
    tests = functiontests(localfunctions);
    run(tests);
else
    test1();
    test2();
    test3();
end

end

function test1(testInput)
import moantool.*
agent = moantool.tests.testAgent1();

%% cannot change the prediction horizon
msg = moantool.utils.run_in_caller('agent.N = 12');
moantool.utils.assert_errmsg(msg, 'You cannot set the read-only property');

%% must not be instantiated by default
assert(~agent.Internal.instantiated);
assert(isempty(agent.X.Var));

%% instantiate() must work
agent.instantiate();
assert(agent.Internal.instantiated);
assert(isa(agent.X.Var, 'ndsdpvar'));
vars = getvariables(agent.X.Var);
% all variables must be unique
assert(isequal(vars, unique(vars)));

%% uninstatiate must work
agent.uninstantiate();
assert(~agent.Internal.instantiated);
assert(~isa(agent.X(1).Var, 'sdpvar'));

%% getParameters must work event without instantiation
params = agent.getParameters();
assert(length(params)==4);

%% getParameters must work after instantiate
agent.instantiate();
params = agent.getParameters();
assert(isa(params, 'containers.Map'));
assert(length(params)==4);

%% optimize() requires an input
msg = moantool.utils.run_in_caller(@() agent.optimize());
moantool.utils.assert_errmsg(msg, 'Not enough input arguments.');

%% optimize() must not work if we have parameters
x0 = zeros(4, 1);
msg = moantool.utils.run_in_caller(@() agent.optimize(x0));
moantool.utils.assert_errmsg(msg, 'Only agents without parameters can be optimized here.');

%% optimizer must work in presence of parameters
[opt, ins, outs] = agent.getOptimizer('gurobi');

%% listParameters must work
T = evalc('agent.listParameters();');
assert(length(T)==112);
% even with a prefix
T = evalc('agent.listParameters(''custom.'');');
assert(length(T)==116);
% even without an empty inputname
a = { agent };
T = evalc('a{1}.listParameters();');
assert(length(T)==104);
end

function test2(testInput)

import moantool.*
agent = moantool.tests.testAgent2();
agent.instantiate();

%% this one has no parameters
params = agent.getParameters();
assert(isempty(params));

%% listParameters must work even if we have no parameters
T = evalc('agent.listParameters()');
assert(isequal(strtrim(T), 'The object has no parameters.'));

%% optimize() must work if we have no parameters
agent.instantiate();
x0 = zeros(4, 1);
[info, Jopt] = agent.optimize(x0);
assert(info.problem==0);
Jexp = 163484.656250074;
assert(norm(Jopt-Jexp, Inf)<1e-4);
Xexp = [0 0.5 1 1.5 2 2 2 2 2 2 1.99995;0 0.0625 0.25 0.5625 1 1.5 2 2.5 3 3.5 3.99999;0 0.5 1 1.5 2 2 2 2 2 2 1.99995;0 0.0625 0.25 0.5625 1 1.5 2 2.5 3 3.5 3.99999];
assert(norm(double(agent.X)-Xexp, Inf)<1e-4);

%% same result with optimizer
[opt, ins, outs] = agent.getOptimizer('gurobi');
[out, problem] = opt{x0};
assert(problem==0);
uexp = [2; 2];
assert(norm(out{1}-uexp, Inf)<1e-4);
assert(isequal(size(out{2}), [4 11])); % X
assert(isequal(size(out{3}), [2 10])); % U
assert(isequal(size(out{4}), [2 10])); % Y
assert(norm(out{2}-Xexp, Inf)<1e-4);

%% optimize() must respect x0
x0 = ones(4, 1);
[info, Jopt] = agent.optimize(x0);
assert(info.problem==0);
Jexp = 145865.453125125;
assert(norm(Jopt-Jexp, Inf)<1e-4);
Xexp = [1 1.5 2 2 2 2 2 2 2 2 1.99994;1 1.3125 1.75 2.25 2.75 3.25 3.75 4.25 4.75 5.25 5.74999;1 1.5 2 2 2 2 2 2 2 2 1.99994;1 1.3125 1.75 2.25 2.75 3.25 3.75 4.25 4.75 5.25 5.74999];
assert(norm(double(agent.X)-Xexp, Inf)<1e-4);

%% problem must be infeasible for some x0
x0 = [18; -10; 18; -10];
[info, Jopt] = agent.optimize(x0);
assert(info.problem==12);

%% problem must be infeasible for some x0 also with optimizer
x0 = [18; -10; 18; -10];
[out, problem] = opt{x0};
assert(problem==12);

%% changes in agent setting must be taken into account
x0 = zeros(4, 1);
agent.Y.Reference = 2*ones(agent.ny, 1);
[info, Jopt] = agent.optimize(x0);
assert(info.problem==0);
Jexp = 322.941374109076;
assert(norm(Jopt-Jexp, Inf)<1e-4);
Xexp = [0 0.5 1 1.45388 1.51505 1.38002 1.18804 1.02502 0.930167 0.90198 0.90198;0 0.0625 0.25 0.556735 0.927851 1.28973 1.61074 1.88737 2.13177 2.36079 2.58629;0 0.5 1 1.45388 1.51505 1.38002 1.18804 1.02502 0.930167 0.90198 0.90198;0 0.0625 0.25 0.556735 0.927851 1.28973 1.61074 1.88737 2.13177 2.36079 2.58629];
assert(norm(double(agent.X)-Xexp, Inf)<1e-4);

%% same result with optimizer
[opt, ins, outs] = agent.getOptimizer('gurobi');
[out, problem] = opt{x0};
assert(problem==0);
assert(norm(out{2}-Xexp, Inf)<1e-4);

%% closed-loop simulation
agent.Y.Reference = [1; -1];
X = zeros(4, 1);
tic
for i = 1:10
    [info, Jopt] = agent.optimize(X(:, end));
    assert(info.problem==0);
    xn = agent.A.Value*X(:, end) + agent.B.Value*double(agent.U, 1);
    X = [X, xn];
end
toc
Xexp = [0 0.5 0.764991 0.799923 0.705666 0.554968 0.3951 0.252886 0.140319 0.0596813 0.00769157;0 0.0625 0.220624 0.416238 0.604437 0.762016 0.880774 0.961773 1.01092 1.03592 1.04434;0 -0.5 -0.764991 -0.799923 -0.705666 -0.554968 -0.3951 -0.252886 -0.140319 -0.0596813 -0.00769157;0 -0.0625 -0.220624 -0.416238 -0.604437 -0.762016 -0.880774 -0.961773 -1.01092 -1.03592 -1.04434];
assert(norm(X-Xexp, Inf)<1e-4);

%% closed-loop simulation with optimizer
agent.Y.Reference = [1; -1];
[opt, ins, outs] = agent.getOptimizer('gurobi');
X = zeros(4, 1);
tic
for i = 1:10
    [out, problem] = opt{X(:, end)};
    assert(problem==0);
    xn = agent.A.Value*X(:, end) + agent.B.Value*out{1};
    X = [X, xn];
end
toc
Xexp = [0 0.5 0.764991 0.799923 0.705666 0.554968 0.3951 0.252886 0.140319 0.0596813 0.00769157;0 0.0625 0.220624 0.416238 0.604437 0.762016 0.880774 0.961773 1.01092 1.03592 1.04434;0 -0.5 -0.764991 -0.799923 -0.705666 -0.554968 -0.3951 -0.252886 -0.140319 -0.0596813 -0.00769157;0 -0.0625 -0.220624 -0.416238 -0.604437 -0.762016 -0.880774 -0.961773 -1.01092 -1.03592 -1.04434];
assert(norm(X-Xexp, Inf)<1e-4);

end

function test3(testInput)
nx=4; nu=3; ny=2; N=10;
agent = moantool.LinearAgent('nx', nx, 'nu', nu, 'ny', ny, 'PredictionHorizon', N);
assert(isequal(agent.A.Dim, [nx nx]));
assert(isequal(agent.B.Dim, [nx nu]));
assert(isequal(agent.f.Dim, [nx 1]));
assert(isequal(agent.C.Dim, [ny nx]));
assert(isequal(agent.D.Dim, [ny nu]));
assert(isequal(agent.g.Dim, [ny 1]));
% wrong inputs must be recognized
msg = moantool.utils.run_in_caller('agent.A.Value=1');
moantool.utils.assert_errmsg(msg, 'Value must be a 4x4 vector/matrix.');
% correct inputs must be allowed
agent.A.Value = eye(nx);
assert(agent.nx==nx);
assert(agent.nu==nu);
assert(agent.ny==ny);
assert(isequal(size(agent.X), [nx 1 N+1]));
assert(isequal(size(agent.U), [nu 1 N]));
assert(isequal(size(agent.Y), [ny 1 N]));
end
