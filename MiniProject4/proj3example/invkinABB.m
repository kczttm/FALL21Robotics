function qsol=invkinABB(robot)
% uses subprob3.m for q3 (2 solns) 
% uses subprob2.m for q1 q2 q4 q5 (2 solns)
% uses subprob1.m for q6 (1 solns)
% uses hat.m to calculate cross product matrix

% Known T try to get q1 q2 q3 q4 q5 q6
% output: qsol = 8(solns) by 6(angles) 
%% Variables
p0T = robot.T(1:3,4);
r0T = robot.T(1:3,1:3);
l = robot.P; %  3xn joint length in its own frame
% postion vectors of the arm
p01 = l(:,1); p12 = l(:,2); p23 = l(:,3); p34 = l(:,4);
p45 = l(:,5); p56 = l(:,6); p6T = l(:,7);
%% Calculations
% a sphere with r = norm(p16)
% a p23 offseted cone of p34 waist height
% intersecting the sphere
% max two sol (maybe infinity?)
p16 = p0T - p01 - r0T*p6T;
q3 = subprob3(robot.H(:,3),-p34,p23,norm(p16));

% two cone with equal waist height intersecting 
% max two sol 
q1 = zeros(length(q3)*2,1);
q2 = zeros(length(q3)*2,1);
q3_holder = q2;  % to expand q3 to 4 by 1
for i = 1:length(q3)
    p24 = p23 + expm(hat(robot.H(:,3))*q3(i))*p34;
    [q1(2*i-1:2*i,:),q2(2*i-1:2*i,:)] = subprob2(-robot.H(:,1),robot.H(:,2),p16,p24);
    q3_holder(2*i-1:2*i,:) = ones(2,1)*q3(i);
end
q3 = q3_holder;

% next, obtain q4 q5 with subproblem2
q4 = zeros(length(q3)*2,1);
q5 = q4;
q123_holder = zeros(length(q3)*2,3);
for i = 1:length(q3)
    r3T = expm(-hat(robot.H(:,3))*q3(i))*expm(-hat(robot.H(:,2))*q2(i))...
        *expm(-hat(robot.H(:,1))*q1(i))*r0T;
    [q4(2*i-1:2*i,:),q5(2*i-1:2*i,:)] = subprob2(-robot.H(:,4),robot.H(:,5),...
        r3T*robot.H(:,6),robot.H(:,6));
    q123_holder(2*i-1:2*i,:) = ones(2,3).*[q1(i),q2(i),q3(i)];
end
q1 = q123_holder(:,1); q2 = q123_holder(:,2); q3 = q123_holder(:,3);

% lastly obtain q6 with subproblem1
q6 = zeros(length(q3),1);
for i = 1:length(q3)
    r5T = expm(-hat(robot.H(:,5))*q5(i))*expm(-hat(robot.H(:,4))*q4(i))...
        *expm(-hat(robot.H(:,3))*q3(i))*expm(-hat(robot.H(:,2))*q2(i))...
        *expm(-hat(robot.H(:,1))*q1(i))*r0T;
    q6(i) = subprob1(robot.H(:,6),robot.H(:,3),r5T*robot.H(:,3));
end
qsol = [q1,q2,q3,q4,q5,q6];
end