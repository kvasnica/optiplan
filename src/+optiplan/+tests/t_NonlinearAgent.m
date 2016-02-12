function t_NonlinearAgent()

if exist('functiontests')
    tests = functiontests(localfunctions);
    run(tests);
else
    test_constructor1;
    test_setStateEq1;
    test_setOutputEq1;
    test_planner_optimize1;
    test_setConstraintsFun1;
    test_setObjectiveFun1;
end

end

function test_constructor1(testInput)
agent = optiplan.NonlinearAgent('nx', 4, 'nu', 2, 'ny', 2, 'PredictionHorizon', 10);
assert(isempty(agent.StateEq));
assert(isempty(agent.OutputEq));
end

function test_setStateEq1(testInput)
agent = optiplan.NonlinearAgent('nx', 4, 'nu', 2, 'ny', 2, 'PredictionHorizon', 10);
% input must be a function handle
msg = optiplan.utils.run_in_caller('agent.StateEq = 1');
optiplan.utils.assert_errmsg(msg, 'The value must be a function handle.');
% the function must take 3 inputs
h = @(x, u) eye(4)*x+ones(4, 2)*u;
msg = optiplan.utils.run_in_caller('agent.StateEq = h');
optiplan.utils.assert_errmsg(msg, 'The function must take 3 inputs: x(k), u(k), agent');
% correct function
h = @(x, u, a) eye(4)*x+ones(4, 2)*u;
agent.StateEq = h;
end

function test_setOutputEq1(testInput)
agent = optiplan.NonlinearAgent('nx', 4, 'nu', 2, 'ny', 2, 'PredictionHorizon', 10);
% input must be a function handle
msg = optiplan.utils.run_in_caller('agent.OutputEq = 1');
optiplan.utils.assert_errmsg(msg, 'The value must be a function handle.');
% the function must take 3 inputs
h = @(x, u) eye(4)*x+ones(4, 2)*u;
msg = optiplan.utils.run_in_caller('agent.OutputEq = h');
optiplan.utils.assert_errmsg(msg, 'The function must take 3 inputs: x(k), u(k), agent');
% correct function
h = @(x, u, a) eye(4)*x+ones(4, 2)*u;
agent.OutputEq = h;
end

function test_setConstraintsFun1(testInput)
agent = optiplan.NonlinearAgent('nx', 4, 'nu', 2, 'ny', 2, 'PredictionHorizon', 10);
% input must be a function handle
msg = optiplan.utils.run_in_caller('agent.ConstraintsFun = 1');
optiplan.utils.assert_errmsg(msg, 'The value must be a function handle.');
% the function must take 4 inputs
h = @(x, u, y) x;
msg = optiplan.utils.run_in_caller('agent.ConstraintsFun = h');
optiplan.utils.assert_errmsg(msg, 'The function must take 4 inputs: X, U, Y, agent');
% correct function
h = @(x, u, y, a) x;
agent.ConstraintsFun = h;
end

function test_setObjectiveFun1(testInput)
agent = optiplan.NonlinearAgent('nx', 4, 'nu', 2, 'ny', 2, 'PredictionHorizon', 10);
% input must be a function handle
msg = optiplan.utils.run_in_caller('agent.ObjectiveFun = 1');
optiplan.utils.assert_errmsg(msg, 'The value must be a function handle.');
% the function must take 4 inputs
h = @(x, u, y) x;
msg = optiplan.utils.run_in_caller('agent.ObjectiveFun = h');
optiplan.utils.assert_errmsg(msg, 'The function must take 4 inputs: X, U, Y, agent');
% correct function
h = @(x, u, y, a) x;
agent.ObjectiveFun = h;
end

function test_planner_optimize1(testInput)
% linear dynamics as a function handle
data = optiplan.LinearAgent.demo2Ddata(0.25);
agent = optiplan.NonlinearAgent('nx', 4, 'nu', 2, 'ny', 2, 'PredictionHorizon', 10);
agent.X.Min = data.xmin;
agent.X.Max = data.xmax;
agent.Y.Min = data.ymin;
agent.Y.Max = data.ymax;
agent.U.Min = data.umin;
agent.U.Max = data.umax;
agent.Y.Penalty = data.Qy;
agent.U.Penalty = data.Qu;
agent.X.Penalty = zeros(4, 4);
agent.Y.Reference = [30; 30];
agent.U.Reference = [0; 0];
agent.X.Reference = zeros(4, 1);
agent.Size.Value = [1; 1];
agent.StateEq = @(x, u, ~) data.A*x+data.B*u;
agent.OutputEq = @(x, u, ~) data.C*x;

obstacles = [];
p = optiplan.Planner(agent, obstacles, 'solver', 'gurobi');
x0 = zeros(4, 1);
[u, info, openloop] = p.optimize(x0);
% outputs must be consistent with t_Planner/test_optimize1
uexp = [2; 2];
Xexp = [0 0.5 1 1.5 2 2 2 2 2 2 1.99995;0 0.0625 0.25 0.5625 1 1.5 2 2.5 3 3.5 3.99999;0 0.5 1 1.5 2 2 2 2 2 2 1.99995;0 0.0625 0.25 0.5625 1 1.5 2 2.5 3 3.5 3.99999];
Uexp = [2 2 2 2 4.91607e-13 -2.48912e-13 -4.48308e-13 -1.04561e-12 -5.21005e-12 -0.000189119;2 2 2 2 4.91607e-13 -2.48912e-13 -4.48308e-13 -1.04561e-12 -5.21005e-12 -0.000189119];
Yexp = [0 0.0625 0.25 0.5625 1 1.5 2 2.5 3 3.5;0 0.0625 0.25 0.5625 1 1.5 2 2.5 3 3.5];
assert(info==0);
assert(norm(uexp-u, Inf)<1e-4);
assert(norm(Uexp-openloop.U, Inf)<5e-4);
assert(norm(Xexp-openloop.X, Inf)<1e-4);
assert(norm(Yexp-openloop.Y, Inf)<1e-4);
end
