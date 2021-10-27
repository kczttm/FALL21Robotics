%
% fwdkinmdh.m
% 

function robot=fwdkinmdh(robot)

ex = [1;0;0]; ez = [0;0;1];

n=length(robot.theta);

T=eye(4,4);
for i=1:n
    T=T*[rotx(robot.alpha(i))*rotz(robot.theta(i)) ...
        robot.d(i)*rotx(robot.alpha(i))*ez+robot.a(i)*ex;...
        [0 0 0 1]];
end

robot.T=T;

end