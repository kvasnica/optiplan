function [agent, obst] = testAgent3(nobst)
% with obstacles

import optiplan.*
if nargin==0
    nobst = 1;
end
% no obstacles by default
agent = optiplan.tests.testAgent2();
% constant penalties
agent.Y.Penalty = 10*eye(agent.ny);
agent.U.Penalty = eye(agent.nu);

% one obstacle
obst = optiplan.Obstacle(agent);
for i = 2:nobst
    % more obstacles
    o = optiplan.Obstacle(agent);
    obst = [obst, o];
end

end
