%
% fwdkinsdh.m
% 

function robot=fwdkinsdh(robot)

ex = [1;0;0]; ez = [0;0;1];

n=length(robot.theta);

T=eye(4,4);
for i=1:n
    T=T*[rotz(robot.theta(i))*rotx(robot.alpha(i)) ...
        robot.d(i)*ez+robot.a(i)*rotz(robot.theta(i))*ex;...
        [0 0 0 1]];
end

robot.T=T;

end