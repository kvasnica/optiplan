function t_Obstacle()
% tests for the Obstacle class

if exist('functiontests')
    tests = functiontests(localfunctions);
    run(tests);
else
    testInput = setupOnce;
    test1(testInput);
    test2(testInput);
    test3(testInput);
    test_objective(testInput);
    test_getParameters(testInput);
    test_getOptimizerData(testInput);
    test_constructorMultiple(testInput);
end

end

function testInput = setupOnce(testCase)
N = 10;
dim = 2;
agent = optiplan.LinearAgent('nx', 4, 'nu', 2, 'ny', dim, 'PredictionHorizon', N);
testCase.TestData.N = N;
testCase.TestData.dim = dim;
testCase.TestData.agent = agent;
testInput.TestData = testCase.TestData;
end

function test1(testInput)
% enough inputs must be provided
msg = optiplan.utils.run_in_caller(@() optiplan.Obstacle());
optiplan.utils.assert_errmsg(msg, 'Not enough input arguments.');
end

function test2(testInput)
% correct signals must be created
N = testInput.TestData.N;
dim = testInput.TestData.dim;
obst = optiplan.Obstacle(testInput.TestData.agent);
assert(isa(obst, 'optiplan.Obstacle'));
assert(length(obst)==1);
expected_signals = {'Position', 'Size', 'Visible'};
expected_dims = {[dim 1], [dim 1], [1 1]};
for i = 1:length(expected_signals)
    sig = obst.(expected_signals{i});
    assert(isa(sig, 'optiplan.AgentSignal'));
    assert(length(sig)==N);
    assert(isequal(sig.Dim, expected_dims{i}));
    assert(sig.hasFilter('Value'));
    f = sig.listFilters();
    assert(length(f)==1);
    assert(isempty(sig.Var));
end
end

function test3(testInput)
% signals must be correctly instantiated
N = testInput.TestData.N;
dim = testInput.TestData.dim;
obst = optiplan.Obstacle(testInput.TestData.agent);
assert(~obst.is_instantiated);
obst.instantiate();
expected_signals = {'Position', 'Size', 'Visible'};
expected_dims = {[dim 1 N], [dim 1 N], [1 1 N]};
for i = 1:length(expected_signals)
    sig = obst.(expected_signals{i});
    assert(~isempty(sig.Var));
    assert(isequal(size(sig.Var), expected_dims{i}));
end

end

function test_objective(testInput)
    
% single obstacle
obst = optiplan.Obstacle(testInput.TestData.agent);
obst.instantiate();
J = obst.objective();
assert(J==0);

% multiple obstacles
o1 = optiplan.Obstacle(testInput.TestData.agent);
o1.instantiate();
o2 = optiplan.Obstacle(testInput.TestData.agent);
o2.instantiate();
O = [o1 o2];
J = O.objective();
assert(J==0);

end

function test_getParameters(testInput)
    
obst = optiplan.Obstacle(testInput.TestData.agent);
p = obst.getParameters();
% all variables are parametric
assert(length(p)==3);
assert(p.isKey('Position.Value'));
assert(p.isKey('Size.Value'));
assert(p.isKey('Visible.Value'));

% now fix 2 of them
obst.Position.Value = [2;2];
obst.Visible.Value = 0;
p = obst.getParameters();
assert(length(p)==1);
assert(~p.isKey('Position.Value'));
assert(p.isKey('Size.Value'));
assert(~p.isKey('Visible.Value'));

end

function test_getOptimizerData(testInput)
    
obst = optiplan.Obstacle(testInput.TestData.agent);

% must not work if not instantiated
msg = optiplan.utils.run_in_caller(@() obst.getOptimizerData());
optiplan.utils.assert_errmsg(msg, 'Object must be instantiated.');

obst.instantiate();
d = obst.getOptimizerData();
assert(isstruct(d));
assert(length(d.inputorder)==3);
assert(length(d.inputs)==3);
assert(isequal(d.inputorder{1}, 'Position.Value'));
assert(isequal(d.inputorder{2}, 'Size.Value'));
assert(isequal(d.inputorder{3}, 'Visible.Value'));
assert(isa(d.inputs{1}, 'sdpvar'));
assert(isa(d.inputs{2}, 'sdpvar'));
assert(isa(d.inputs{3}, 'sdpvar'));
assert(isequal(size(d.inputs{1}), [2 10]));
assert(isequal(size(d.inputs{2}), [2 10]));
assert(isequal(size(d.inputs{3}), [1 10]));
assert(isempty(d.outputs));
assert(isempty(d.outputorder));

end

function test_constructorMultiple(testInput)

nobst = 3;
obst = optiplan.Obstacle(testInput.TestData.agent, nobst);
assert(length(obst)==nobst);
for i = 1:nobst, assert(isa(obst(i), 'optiplan.Obstacle')); end

end
