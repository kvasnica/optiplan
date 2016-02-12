function t_AgentSignal()
% tests for the AgentSignal class

import optiplan.*
yalmip clear

%% first input must be a 1x2 array of dimensions
msg = optiplan.utils.run_in_caller(@() optiplan.AgentSignal(1));
optiplan.utils.assert_errmsg(msg, 'The first input must be a 1x2 vector.');

%% correct inputs must be accepted
n = 3; m = 5;
z = AgentSignal([n 1], m);

%% correct size?
assert(length(z)==m);
assert(isequal(size(z), [n 1 m]));

%% no name by default
assert(isempty(z.Name));

%% name must be set
z = AgentSignal([n 1], m, 'myname');
assert(isequal(z.Name, 'myname'));

%% correct default values?
assert(isequal(z.Min, 'parameter'));
assert(isequal(z.Max, 'parameter'));
assert(isequal(z.Penalty, 'parameter'));
assert(isequal(z.Reference, 'parameter'));

%% can set doubles/strings?
v = ones(n, 1); M = eye(n);
z.Min = v; assert(isequal(z.Min, v));
z.Max = v; assert(isequal(z.Max, v));
z.Reference = v; assert(isequal(z.Reference, v));
z.Penalty = M; assert(isequal(z.Penalty, M));
% set back to parameters
v = 'parameter'; M = 'parameter';
z.Min = v; assert(isequal(z.Min, v));
z.Max = v; assert(isequal(z.Max, v));
z.Reference = v; assert(isequal(z.Reference, v));
z.Penalty = M; assert(isequal(z.Penalty, M));

%% AgentSignal.squeeze() works with matrices?
k = 2;
z = AgentSignal([n k], m);
z.instantiate();
v = z.squeeze();
assert(isa(v, 'sdpvar'));
assert(size(v, 1)==n);
assert(size(v, 2)==k*m);

%% AgentSignal.squeeze() works with vectors?
z = AgentSignal([n 1], m);
z.instantiate();
v = z.squeeze();
assert(isa(v, 'sdpvar'));
assert(size(v, 1)==n);
assert(size(v, 2)==m);

%% must not be instantiated at the beginning
z = AgentSignal([n 1], m);
assert(isempty(z.Var));

%% must not uninstantiate if it was not instantiated before
z = optiplan.AgentSignal([n 1], m);
msg = optiplan.utils.run_in_caller(@() z.uninstantiate);
optiplan.utils.assert_errmsg(msg, 'Signal is not instantiated.');

%% must instantiate properly
z.instantiate();
assert(~isempty(z.Var));
assert(isa(z.Var, 'ndsdpvar'));
assert(isequal(size(z.Var), [n 1 m]));
assert(length(getvariables(z.Var))==n*1*m);

%% must uninstantiate properly
z.uninstantiate();
assert(isempty(z.Var));

%% no constraints() without instantiate()
msg = optiplan.utils.run_in_caller(@() z.constraints);
optiplan.utils.assert_errmsg(msg, 'Signal is not instantiated.');

%% constraints() must work after instantiate()
z = AgentSignal([n, 1], m);
z.instantiate();
C = z.constraints();
% 2 constraints? (one for min, one for max)
assert(length(C)==2);
% on 2*n*1*m separate variables?
assert(length(sdpvar(C))==2*n*1*m);
% are parametric variables included? "n*1*m" for the signal, "n*1*m" for min, "n*1*m" for max
assert(length(getvariables(C))==3*n*1*m);
% are the signal's variables included?
assert(all(ismember(getvariables(z.Var), getvariables(C))));
% are parametric upper bounds included?
assert(all(ismember(getvariables(z.Internal.parameters.Min), getvariables(C))));
% are parametric lower bounds included?
assert(all(ismember(getvariables(z.Internal.parameters.Max), getvariables(C))));
% are all variables unique?
assert(isequal(unique(getvariables(C)), getvariables(C)));

%% no objective() without instantiate();
z = AgentSignal([n, 1], m);
msg = optiplan.utils.run_in_caller(@() z(1).objective);
optiplan.utils.assert_errmsg(msg, 'Signal is not instantiated.');

%% objective() must work after instantiate() (constant penalty)
z.Penalty = eye(n);
z.instantiate();
obj = z(1).objective();
assert(isa(obj, 'sdpvar'));
assert(length(getvariables(obj))==n^2*m); % for a constant penalty

%% objective() must work after instantiate() (parametric penalties)
z.uninstantiate();
z.Penalty = 'parameter';
z.instantiate();
obj = z.objective();
assert(isa(obj, 'sdpvar'));

%% getParameters() must work even without instantiate()
z = AgentSignal([n, 1], m);
z.Max = ones(n, 1);
params = z.getParameters();
assert(isa(params, 'containers.Map'));
assert(isempty(params('Min').Var));
assert(isequal(params('Min').Dim, [n 1 m]));
assert(isempty(params('Penalty').Var));
assert(isequal(params('Penalty').Dim, [n n m]));
assert(isempty(params('Reference').Var));
assert(isequal(params('Reference').Dim, [n 1 m]));
assert(~params.isKey('Max')); % becuase it's not a parameter

%% getParameters() after instantiate() must return variables
z.instantiate();
params = z.getParameters();
assert(isa(params, 'containers.Map'));
assert(isa(params('Min').Var, 'ndsdpvar'));
assert(isequal(params('Min').Dim, [n 1 m]));
assert(isa(params('Penalty').Var, 'ndsdpvar'));
assert(isequal(params('Penalty').Dim, [n n m]));
assert(isa(params('Reference').Var, 'ndsdpvar'));
assert(isequal(params('Reference').Dim, [n 1 m]));
assert(~params.isKey('Max')); % becuase its not a parameter

%% test correctness
% min  x'*P*x
% s.t. a <= x <= b

%% constant "a", "b", "P", xopt = b
n = 2; z = AgentSignal([n 1], 1);
a = [-1; -2]; b = [-0.5; -0.4]; P = eye(2);
z.Min = a;
z.Max = b;
z.Penalty = P;
z.Reference = zeros(2, 1);
z.instantiate();
z.optimize()
xopt = b; Jopt = xopt'*P*xopt;
assert(norm(value(z.Var)-xopt, Inf)<1e-5);

%% constant "a", "b", "P", xopt = a
n = 2; z = AgentSignal([n 1], 1);
a = [0.5; 0.4]; b = [1; 2]; P = eye(2);
z.Min = a;
z.Max = b;
z.Penalty = P;
z.Reference = zeros(2, 1);
z.instantiate();
z.optimize()
xopt = a; Jopt = xopt'*P*xopt;
assert(norm(value(z.Var)-xopt, Inf)<1e-5);

%% constant "a", "b", "P", xopt = xref
n = 2; z = AgentSignal([n 1], 1);
a = [-0.5; -0.4]; b = [1; 2]; P = eye(2);
z.Min = a;
z.Max = b;
z.Penalty = P;
z.Reference = [0.1; 0.1];
z.instantiate();
z.optimize()
xopt = z.Reference; Jopt = xopt'*P*xopt;
assert(norm(value(z.Var)-xopt, Inf)<1e-5);

%% parametric a, b, fixed P, reference
n = 2; z = AgentSignal([n 1], 1);
opts = sdpsettings('verbose', 0);
z.Penalty = P;
z.Reference = [0.1; 0.1];
z.instantiate();
C = z.constraints();
obj = z.objective();
a = [0.5; 0.4]; b = [1; 2]; xopt = a;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
a = [-1; -2]; b = [-0.5; -0.4]; xopt = b;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
a = [-1; -2]; b = [0.5; 0.4]; xopt = z.Reference;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);

%% parametric a, b, reference, fixed P
n = 2; z = AgentSignal([n 1], 1);
opts = sdpsettings('verbose', 0);
z.Penalty = P;
z.instantiate();
C = z.constraints();
obj = z.objective();
%
a = [0.5; 0.4]; b = [1; 2]; ref = [0; 0]; xopt = a;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
%
a = [0.5; 0.4]; b = [1; 2]; ref = [2; 2]; xopt = b;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
%
a = [0.5; 0.4]; b = [1; 2]; ref = [1; 1]; xopt = ref;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
%
a = [-1; -2]; b = [-0.5; -0.4]; ref = [0; 0]; xopt = b;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
%
a = [-1; -2]; b = [-0.5; -0.4]; ref = [-2; -2]; xopt = a;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
%
a = [-1; -2]; b = [0.5; 0.4]; ref = [0; 0]; xopt = ref;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
%
a = [-1; -2]; b = [0.5; 0.4]; ref = [2; 2]; xopt = b;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
%
a = [-1; -2]; b = [0.5; 0.4]; ref = [-2; -2]; xopt = a;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
%
a = [-1; -2]; b = [0.5; 0.4]; ref = [0.1; 0.1]; xopt = ref;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);

%% parametric a, b, reference, P
n = 2; z = AgentSignal([n 1], 1);
opts = sdpsettings('verbose', 0);
z.Penalty = P;
z.instantiate();
C = z.constraints();
obj = z.objective();
%
a = [0.5; 0.4]; b = [1; 2]; ref = [0; 0]; xopt = a;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
%
a = [0.5; 0.4]; b = [1; 2]; ref = [2; 2]; xopt = b;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
%
a = [0.5; 0.4]; b = [1; 2]; ref = [1; 1]; xopt = ref;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
%
a = [-1; -2]; b = [-0.5; -0.4]; ref = [0; 0]; xopt = b;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
%
a = [-1; -2]; b = [-0.5; -0.4]; ref = [-2; -2]; xopt = a;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
%
a = [-1; -2]; b = [0.5; 0.4]; ref = [0; 0]; xopt = ref;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
%
a = [-1; -2]; b = [0.5; 0.4]; ref = [2; 2]; xopt = b;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
%
a = [-1; -2]; b = [0.5; 0.4]; ref = [-2; -2]; xopt = a;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);
%
a = [-1; -2]; b = [0.5; 0.4]; ref = [0.1; 0.1]; xopt = ref;
optimize(C+[z.Internal.parameters.Min==a; z.Internal.parameters.Max==b; z.Internal.parameters.Reference==ref], obj, opts);
assert(norm(value(z.Var)-xopt, Inf)<1e-5);

%% test optimizer
n = 2; z = AgentSignal([n 1], 1);
z.instantiate();
[opt, order] = z.getOptimizer('gurobi');
assert(isequal(order{1}, 'Max'));
assert(isequal(order{2}, 'Min'));
assert(isequal(order{3}, 'Penalty'));
assert(isequal(order{4}, 'Reference'));
%
zmin = [-1; -1]; zmax = [2; 1]; zP = eye(2); zref = [0; 0]; xopt = zref;
zopt = opt{{zmax, zmin, zP, zref}}; assert(norm(zopt - xopt, Inf)<1e-5);
%
zmin = [-1; -1]; zmax = [2; 1]; zP = eye(2); zref = [0.1; 0.1]; xopt = zref;
zopt = opt{{zmax, zmin, zP, zref}}; assert(norm(zopt - xopt, Inf)<1e-5);
%
zmin = [-1; -1]; zmax = [2; 1]; zP = eye(2); zref = zmin; xopt = zmin;
zopt = opt{{zmax, zmin, zP, zref}}; assert(norm(zopt - xopt, Inf)<1e-5);
%
zmin = [-1; -1]; zmax = [2; 1]; zP = eye(2); zref = [-10; -10]; xopt = zmin;
zopt = opt{{zmax, zmin, zP, zref}}; assert(norm(zopt - xopt, Inf)<1e-5);
%
zmin = [-1; -1]; zmax = [2; 1]; zP = eye(2); zref = zmax; xopt = zmax;
zopt = opt{{zmax, zmin, zP, zref}}; assert(norm(zopt - xopt, Inf)<1e-5);
%
zmin = [-1; -1]; zmax = [2; 1]; zP = [1 0.5; 0.5 1]; zref = [0.5; 1]; xopt = zref;
zopt = opt{{zmax, zmin, zP, zref}}; assert(norm(zopt - xopt, Inf)<1e-4);

%% test the "value" filter
n = 3; f = AgentSignal([n 1], 1);
% the filter is not enabled by default
assert(~f.hasFilter('Value'));
% can be added manually
f.with('Value');
assert(f.hasFilter('Value'));
% setter must check dimensions
msg = optiplan.utils.run_in_caller('f.Value=ones(2*n, 1)');
optiplan.utils.assert_errmsg(msg, sprintf('Value must be a %dx1', n));
% correct inputs must be allowed
f.Value = ones(n, 1);
v = f.value_or_var();
assert(isequal(v, ones(n, 1)));
% parametric value must return sdpvar
f.Value = 'parameter';
f.instantiate();
v = f.value_or_var();
assert(isa(v, 'sdpvar'));
assert(isequal(size(v), [n 1]));

%% matrix signals
n = 2; m = 4; f = AgentSignal([n 1], m);
f.with('Value');
assert(isequal(f.Dim, [n 1]));
assert(length(f)==m);
% setter must check dimensions
msg = optiplan.utils.run_in_caller('f.Value=1');
optiplan.utils.assert_errmsg(msg, sprintf('Value must be a %dx%d', n, 1));
% setter must allow correct inputs
r = rand(n, 1);
f.Value = r;
v = f.value_or_var();
assert(isequal(v, r));
% now a parameter
f.Value = 'parameter';
% before instantiation
v = f.value_or_var();
assert(isempty(v));
% after instantiation
f.instantiate();
v = f.value_or_var();
assert(isa(v, 'ndsdpvar'));
assert(isequal(size(v), [n 1 m]));
% correct variables must be created
assert(isequal(size(f.Internal.parameters.Max), [n 1 m]));
assert(isequal(size(f.Internal.parameters.Min), [n 1 m]));
assert(isequal(size(f.Internal.parameters.Reference), [n 1 m]));
assert(isequal(size(f.Internal.parameters.Penalty), [n n m]));

% x = sdpvar(2, 1); optimize([zmin<=x<=zmax], (x-zref)'*zP*(x-zref)); value(x)
end
