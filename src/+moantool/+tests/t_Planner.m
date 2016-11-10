function t_Planner()

if exist('functiontests')
    tests = functiontests(localfunctions);
    run(tests);
else
    
    test_constructor1();
    test_constructor2();
    test_constructor3;
    test_constructor4;
    test_constructor5;
    test_constructor6;
    test_readParameters1;
    test_getParameters1;
    test_listParameters1;
    test_listMissing1;
    test_optimize1;
    test_optimize2;
    test_optimize3;
    test_optimize4;
    test_optimize5;
    test_obstacles1;
    test_obstacles2;
    test_obstacles3;
    test_obstacles4;
    test_obstacles5;
end

end

function test_constructor1(testInput)
% first input must be Agent
err = moantool.utils.run_in_caller(@() moantool.Planner([]));
moantool.utils.assert_errmsg(err, 'Not enough input arguments.');
end

function test_constructor2(testInput)
% second input must be either empty or Obstacle
agent = moantool.LinearAgent('nx', 4, 'nu', 2, 'ny', 2, 'PredictionHorizon', 10);
obst = 1;
err = moantool.utils.run_in_caller(@() moantool.Planner(agent, obst));
moantool.utils.assert_errmsg(err, 'The second input must be an Obstacle object.');
end

function test_constructor3(testInput)
% constructor must work with empty obstacles and no solver
agent = moantool.tests.testAgent1();
obstacles = [];
p = moantool.Planner(agent, obstacles);
end

function test_constructor4(testInput)
% constructor must work with empty obstacles
agent = moantool.tests.testAgent1();
obstacles = [];
p = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
end

function test_constructor5(testInput)
% constructor must work with one obstacle
agent = moantool.tests.testAgent1();
obstacles = moantool.Obstacle(agent);
p = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
end

function test_constructor6(testInput)
% constructor must work with many obstacles
agent = moantool.tests.testAgent1();
obstacles = [moantool.Obstacle(agent); moantool.Obstacle(agent); moantool.Obstacle(agent);];
p = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
assert(length(p.Parameters.Obstacles)==length(obstacles));
assert(isstruct(p.Parameters.Agent));
for i = 1:length(obstacles)
    assert(isstruct(p.Parameters.Obstacles(i)));
end
end

function test_getParameters1(testInput)
% parameters must be detected
agent = moantool.tests.testAgent1();
obstacles = [moantool.Obstacle(agent); moantool.Obstacle(agent); moantool.Obstacle(agent);];
obstacles(2).Size.Value = [1; 1];
p = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
params = p.getParameters();
assert(length(params)==12);
end

function test_readParameters1(testInput)
% values must be checked for correct size
agent = moantool.tests.testAgent1();
obstacles = [];
p = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
p.readParameters(); % must work with default NaN setting
p.Parameters.Agent.Size.Value = zeros(2, 3); % wrong size
msg = moantool.utils.run_in_caller(@() p.readParameters());
moantool.utils.assert_errmsg(msg, 'Parameter.Agent(1).Size.Value must be either a 2x1 or a 2x10 matrix.');
end

function test_listParameters1(testInput)
% parameters must be displayed correctly
agent = moantool.tests.testAgent1();
obstacles = [moantool.Obstacle(agent); moantool.Obstacle(agent); moantool.Obstacle(agent);];
p = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
T = evalc('p.listParameters();');
assert(length(T)==663);
end

function test_listMissing1(testInput)
% missing values must be detected
agent = moantool.tests.testAgent1();
obstacles = [moantool.Obstacle(agent); moantool.Obstacle(agent); moantool.Obstacle(agent);];
p = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
missing = p.listMissing();
assert(length(missing)==13);
% now set some values, they must be recognized
p.Parameters.Agent.Y.Max = [1; 1];
p.Parameters.Agent.Y.Reference = [0; 0];
missing = p.listMissing();
assert(length(missing)==11);
end

function test_optimize1(testInput)
% optimize() must work even in absence of parameters
agent = moantool.tests.testAgent2(); % no obstacles
obstacles = [];
p = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
x0 = zeros(4, 1);
[u, info, openloop] = p.optimize(x0);
uexp = [2; 2];
Xexp = [0 0.5 1 1.5 2 2 2 2 2 2 1.99995;0 0.0625 0.25 0.5625 1 1.5 2 2.5 3 3.5 3.99999;0 0.5 1 1.5 2 2 2 2 2 2 1.99995;0 0.0625 0.25 0.5625 1 1.5 2 2.5 3 3.5 3.99999];
Uexp = [2 2 2 2 4.91607e-13 -2.48912e-13 -4.48308e-13 -1.04561e-12 -5.21005e-12 -0.000189119;2 2 2 2 4.91607e-13 -2.48912e-13 -4.48308e-13 -1.04561e-12 -5.21005e-12 -0.000189119];
Yexp = [0 0.0625 0.25 0.5625 1 1.5 2 2.5 3 3.5;0 0.0625 0.25 0.5625 1 1.5 2 2.5 3 3.5];
assert(info==0);
assert(norm(uexp-u, Inf)<1e-4);
assert(norm(Uexp-openloop.U, Inf)<1e-4);
assert(norm(Xexp-openloop.X, Inf)<1e-4);
assert(norm(Yexp-openloop.Y, Inf)<1e-4);
end

function test_optimize2(testInput)
% ymin/ymax bounds must be tightened w.r.t. agent's size
agent = moantool.tests.testAgent2(); % no obstacles
agent.Size.Value = [6; 8];
agent.Y.Reference = [30; 30];
obstacles = [];
p = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
x0 = [0; 15; 0; 14];
[u, info, openloop] = p.optimize(x0);
yexp = agent.Y(1).Max-agent.Size(1).Value/2;
assert(norm(openloop.Y(:, end)-yexp, Inf)<1e-4);
%
agent.Size.Value = [6; 8];
agent.Y.Reference = -[30; 30];
obstacles = [];
p = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
x0 = -[0; 15; 0; 14];
[u, info, openloop] = p.optimize(x0);
yexp = agent.Y(1).Min+agent.Size(1).Value/2;
assert(norm(openloop.Y(:, end)-yexp, Inf)<1e-4);
end

function test_optimize3(testInput)
% optimize() must work with parameters (no obstacles)
obstacles = [];
a1 = moantool.tests.testAgent1();
a2 = moantool.tests.testAgent2();
p = moantool.Planner(a1, obstacles, 'solver', 'gurobi');
x0 = zeros(4, 1);
% also tests automatic expansion of horizon-1 values to horizon-N
p.Parameters.Agent.U.Min = a2.U.Min;
p.Parameters.Agent.U.Max = a2.U.Max;
p.Parameters.Agent.Y.Min = a2.Y.Min;
p.Parameters.Agent.Y.Max = a2.Y.Max;
p.Parameters.Agent.Y.Reference = a2.Y.Reference;
p.Parameters.Agent.Size.Value = [1; 1];
m = p.listMissing();
assert(isempty(m));
[u, info, openloop] = p.optimize(x0);
uexp = [2; 2];
Xexp = [0 0.5 1 1.5 2 2 2 2 2 2 1.99995;0 0.0625 0.25 0.5625 1 1.5 2 2.5 3 3.5 3.99999;0 0.5 1 1.5 2 2 2 2 2 2 1.99995;0 0.0625 0.25 0.5625 1 1.5 2 2.5 3 3.5 3.99999];
Uexp = [2 2 2 2 4.91607e-13 -2.48912e-13 -4.48308e-13 -1.04561e-12 -5.21005e-12 -0.000189119;2 2 2 2 4.91607e-13 -2.48912e-13 -4.48308e-13 -1.04561e-12 -5.21005e-12 -0.000189119];
Yexp = [0 0.0625 0.25 0.5625 1 1.5 2 2.5 3 3.5;0 0.0625 0.25 0.5625 1 1.5 2 2.5 3 3.5];
assert(info==0);
assert(norm(uexp-u, Inf)<1e-5);
assert(norm(Uexp-openloop.U, Inf)<1e-3);
assert(norm(Xexp-openloop.X, Inf)<1e-3);
assert(norm(Yexp-openloop.Y, Inf)<1e-3);
end

function test_optimize4(testInput)
% closed-loop simulation (no obstacles)
a2 = moantool.tests.testAgent2();
yref = [1; -1];
a2.Y.Reference = yref;
obstacles = [];
p = moantool.Planner(a2, obstacles, 'solver', 'gurobi');
x0 = zeros(4, 1);
X = x0;
U = [];
for i = 1:50
    x = X(:, end);
    [u, problem] = p.optimize(x);
    assert(problem==0);
    xn = p.Agent.A(1).Value*x+p.Agent.B(1).Value*u;
    X = [X, xn];
    U = [U, u];
end
xref = [0; yref(1); 0; yref(2)];
assert(norm(X(:, end)-xref, Inf)<1e-3);
end

function test_optimize5(testInput)
% closed-loop simulation with time-varying reference (no obstacles)
a2 = moantool.tests.testAgent2();
a2.Y.Reference = 'parameter';
obstacles = [];
p = moantool.Planner(a2, obstacles, 'solver', 'gurobi');
x0 = zeros(4, 1);
X = x0;
U = [];
ref1 = [1; -1];
ref2 = [-1; 1];
for i = 1:100
    if i<=50
        p.Parameters.Agent.Y.Reference = ref1;
    else
        p.Parameters.Agent.Y.Reference = ref2;
    end
    x = X(:, end);
    [u, problem] = p.optimize(x);
    assert(problem==0);
    xn = p.Agent.A(1).Value*x+p.Agent.B(1).Value*u;
    X = [X, xn];
    U = [U, u];
end
assert(norm(X([2 4], 50)-ref1, Inf)<1e-3);
assert(norm(X([2 4], end)-ref2, Inf)<1e-3);
end

function test_obstacles1(testInput)
% static non-parametric obstacle
% size of the obstacle
osx = 5; osy = 2;
% size of the agent
asx = 1; asy = 0.5;
% obstacle avoidance with 1 obstacle
[agent, obstacles] = moantool.tests.testAgent3(1);
% constant position of the obstacle
obstacles(1).Position.Value = [0; 0];
% constant size
obstacles(1).Size.Value = [osx; osy]*2;
% constant visibility
obstacles(1).Visible.Value = 1;
% constant geometry
agent.Size.Value = [asx; asy];
% constant references
agent.Y.Reference = [0; 0];
planner = moantool.Planner(agent, obstacles, 'solver', 'gurobi');
% colliding initial condition
x0 = [0; osx+asx-0.1; 0; osy+asy-0.1]/2;
[u, prob, ol] = planner.optimize(x0);
assert(prob==12); % must be infeasible
% collision-free initial condition
x0 = [0; -0.5; 0; osy+asy+0.1];
y0 = [x0(2); x0(4)];
[u, prob, ol] = planner.optimize(x0);
assert(prob==0); % must be feasible
uexp = [1.13133;-2];
assert(norm(uexp-u, Inf)<1e-4);
Uexp = [1.13133 0.443808 0.0202592 -0.196708 -0.267296 -0.245452 -0.175583 -0.0929792 -0.0261954 -5.1914e-13;-2 -0.350995 1.13402 1.15292 0.128117 -0.108258 0.0685395 -0.0486803 0.0486803 -9.19487e-13];
assert(norm(Uexp-ol.U, Inf)<1e-4);
Xexp = [0 0.282832 0.393784 0.398849 0.349672 0.282848 0.221485 0.177589 0.154344 0.147795 0.147795;-0.5 -0.464646 -0.380069 -0.28099 -0.187425 -0.10836 -0.0453181 0.00456614 0.0460578 0.0838253 0.120774;0 -0.5 -0.587749 -0.304244 -0.0160146 0.0160146 -0.0110498 0.00608504 -0.00608504 0.00608504 0.00608504;2.6 2.5375 2.40153 2.29003 2.25 2.25 2.25062 2.25 2.25 2.25 2.25152];
assert(norm(Xexp-ol.X, Inf)<1e-4);
Yexp = [-0.5 -0.464646 -0.380069 -0.28099 -0.187425 -0.10836 -0.0453181 0.00456614 0.0460578 0.0838253;2.6 2.5375 2.40153 2.29003 2.25 2.25 2.25062 2.25 2.25 2.25];
assert(norm(ol.Y(:, 1)-y0, Inf)<1e-4);
assert(norm(Yexp-ol.Y, Inf)<1e-4);
end

function test_obstacles2(testInput)
% single fixed obstacle

% size of the obstacle
osx = 5; osy = 2;
% size of the agent
asx = 1; asy = 0.5;
% obstacle avoidance with 1 obstacle
[agent, obstacles] = moantool.tests.testAgent3(1);
assert(length(obstacles)==1);
% constant position of the obstacle
obstacles(1).Position.Value = [0; 0];
% constant size
obstacles(1).Size.Value = [osx; osy];
% constant visibility
obstacles(1).Visible.Value = 1;
% constant geometry
agent.Size.Value = [asx; asy];
% constant references
agent.Y.Reference = [0; 0];
% planner
planner = moantool.Planner(agent, obstacles);
% not parameters must be left
params = planner.getParameters();
assert(isempty(params));

% points to test for infeasibility (collision by construction)
P = [0 0; osx osy; -osx osy; osx -osy; -osx -osy];
% must be infeasible also outside obstacle since the agent has
% non-zero size
P = [P; osx+asx/2 osy; -osx-asx/2 osy; osx osy+asy/2; osx osy-asy/2];
P = P/2; % because the sizes are the overall width/height
for i = 1:size(P, 1)
    y0 = P(i, :)'; x0 = [0; y0(1); 0; y0(2)];
    [~, prob] = planner.optimize(x0);
    assert(prob==12); % must be infeasible
end
% must be feasible outside of the collision zone
P = [osx+asx+0.1 0; -(osx+asx+0.1) 0; 0 osy+asy+0.1; 0 -(osy+asy+0.1)];
P = [P; osx+asx+0.1 osy+asy+0.1; osx+asx+0.1 -(osy+asy+0.1)];
P = [P; -(osx+asx+0.1) osy+asy+0.1; -(osx+asx+0.1) -(osy+asy+0.1)];
P = P/2; % because the sizes are the overall width/height
for i = 1:size(P, 1)
    y0 = P(i, :)'; x0 = [0; y0(1); 0; y0(2)];
    [~, prob] = planner.optimize(x0);
    assert(prob==0); % must be feasible
end
end

function test_obstacles3(testInput)
% obstacle avoidance with multiple obstacles
nobst = 3;
[agent, obstacles] = moantool.tests.testAgent3(nobst);
planner = moantool.Planner(agent, obstacles);
c = planner.constraints();
assert(length(c)==256);
J = planner.objective();
end

function test_obstacles4(testInput)
% invisible obstacles should be ignored (hard-coded invisibility)

% size of the obstacle
osx = 5; osy = 2;
% size of the agent
asx = 1; asy = 0.5;
% obstacle avoidance with 1 obstacle
[agent, obstacles] = moantool.tests.testAgent3(1);
% constant position of the obstacle
obstacles(1).Position.Value = [0; 0];
% constant size
obstacles(1).Size.Value = [osx; osy];
% INVISIBLE obstacle
obstacles(1).Visible.Value = 0;
% constant geometry
agent.Size(1).Value = [asx; asy];
% planner
planner = moantool.Planner(agent, obstacles);
% not parameters must be left
params = planner.getParameters();

% points to test for infeasibility (no collision because the
% obstacle is invisible)
P = [0 0; osx osy; -osx osy; osx -osy; -osx -osy];
% must be infeasible also outside obstacle since the agent has
% non-zero size
P = [P; osx+asx/2 osy; -osx-asx/2 osy; osx osy+asy/2; osx osy-asy/2];
for i = 1:size(P, 1)
    y0 = P(i, :)'; x0 = [0; y0(1); 0; y0(2)];
    [~, prob] = planner.optimize(x0);
    assert(prob==0); % must feasible because the obstacle is invisible
end
% must be feasible outside of the collision zone
P = [osx+asx+0.1 0; -(osx+asx+0.1) 0; 0 osy+asy+0.1; 0 -(osy+asy+0.1)];
P = [P; osx+asx+0.1 osy+asy+0.1; osx+asx+0.1 -(osy+asy+0.1)];
P = [P; -(osx+asx+0.1) osy+asy+0.1; -(osx+asx+0.1) -(osy+asy+0.1)];
for i = 1:size(P, 1)
    y0 = P(i, :)'; x0 = [0; y0(1); 0; y0(2)];
    [~, prob] = planner.optimize(x0);
    assert(prob==0); % must be feasible because we are outside of the collision zone
end
end


function test_obstacles5(testInput)
% invisible obstacles should be ignored (parametric invisibility)

% size of the obstacle
osx = 5; osy = 2;
% size of the agent
asx = 1; asy = 0.5;
% obstacle avoidance with 1 obstacle
[agent, obstacles] = moantool.tests.testAgent3(1);
% constant position of the obstacle
obstacles(1).Position.Value = [0; 0];
% constant size
obstacles(1).Size.Value = [osx; osy];
% constant geometry
agent.Size(1).Value = [asx; asy];
% planner
planner = moantool.Planner(agent, obstacles);
% must have 1 parameter (the visibility)
assert(length(planner.getParameters)==1);

% parametrically set the obstacle to be invisible
planner.Parameters.Obstacles(1).Visible.Value = 0;
% planner.Parameters.Obstacles.Visible(3).Value = 'parameter';

% points to test for infeasibility (no collision because the
% obstacle is invisible)
P = [0 0; osx osy; -osx osy; osx -osy; -osx -osy];
% must be infeasible also outside obstacle since the agent has
% non-zero size
P = [P; osx+asx/2 osy; -osx-asx/2 osy; osx osy+asy/2; osx osy-asy/2];
for i = 1:size(P, 1)
    y0 = P(i, :)'; x0 = [0; y0(1); 0; y0(2)];
    [~, prob] = planner.optimize(x0);
    assert(prob==0); % must feasible because the obstacle is invisible
end
% must be feasible outside of the collision zone
P = [osx+asx+0.1 0; -(osx+asx+0.1) 0; 0 osy+asy+0.1; 0 -(osy+asy+0.1)];
P = [P; osx+asx+0.1 osy+asy+0.1; osx+asx+0.1 -(osy+asy+0.1)];
P = [P; -(osx+asx+0.1) osy+asy+0.1; -(osx+asx+0.1) -(osy+asy+0.1)];
for i = 1:size(P, 1)
    y0 = P(i, :)'; x0 = [0; y0(1); 0; y0(2)];
    [~, prob] = planner.optimize(x0);
    assert(prob==0); % must be feasible because we are outside of the collision zone
end
end
