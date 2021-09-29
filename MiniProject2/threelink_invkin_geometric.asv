function qsol=threelink_invkin_geometric(robot)
% uses subprob3.m for two circle intersection to get q2 
% uses subprob1.m to get angle between two known vector
% Known T try to get q1 q2 q3
%% Variables
p0T = robot.T(1:3,4);
r0T = robot.T(1:3,1:3);
l = robot.P(:,2:end); %  3xn joint length in its own frame
% postion vectors of the arm
p01 = robot.P(:,1); p12 = l(:,1); p23 = l(:,2); p3T = l(:,3); 

%% Calculations
k = robot.H(:,1); % so far this only works for planar case - same rotation axis rn
qt =subprob1(k,[1;0;0],r0T(:,1)); % q1 + q2 + q3 = qt
p13 = p0T - p01 - rot(k,qt)*p3T;
d = norm(p13);
q2=subprob3(k,-p23,p12,d); 
q1(1) = subprob1(k,p12+rot(k,q2(1))*p23,p13);
q1(2) = subprob1(k,p12+rot(k,q2(2))*p23,p13);
qt = [qt,qt];
q3 = qt - q1 - q2;
qsol = wrapToPi([q1;q2;q3]);
end