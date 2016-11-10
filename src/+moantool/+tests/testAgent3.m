function [agent, obst] = testAgent3(nobst)
% with obstacles

import moantool.*
if nargin==0
    nobst = 1;
end
% no obstacles by default
agent = moantool.tests.testAgent2();
% constant penalties
agent.Y.Penalty = 10*eye(agent.ny);
agent.U.Penalty = eye(agent.nu);

% one obstacle
obst = moantool.Obstacle(agent);
for i = 2:nobst
    % more obstacles
    o = moantool.Obstacle(agent);
    obst = [obst, o];
end

end
