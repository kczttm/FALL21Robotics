%
% checking three-link arm forward and inverse kinematics
%

%
% define 3-link arm
%

n_link = 3; % 3 links
zz = [0;0;0]; ex=[1;0;0]; ez=[0;0;1];
% first check symbolically
syms q1 q2 q3 real
syms l1 l2 l3 real
robotsym.P = [zz l1*ex l2*ex l3*ex];
robotsym.H = [ez ez ez];
robotsym.q=[q1;q2;q3];
robotsym.joint_type=[0 0 0];
% caculate forward kinematics 
robotsym=nlinkfwdkin(robotsym);
% display final orientation and position
disp('R0T');disp(simplify(robotsym.T(1:2,1:2)));
disp('p0T');disp(simplify(robotsym.T(1:2,4)));
% display Jacobian
%disp('JT');disp(simplify(robotsym.J(1:3,1:3)));
%disp('det(JT)');disp(simplify(det(robotsym.J(1:3,1:3))));

% robot parameters
l1 = 2; l2 = 2; l3 = 0.5;
robot.P = [zz l1*ex l2*ex l3*ex];
robot.H = [ez ez ez];
robot.joint_type=[0 0 0];
%robot.q=zeros(3,1);
robot.q=(rand(3,1)-.5)*2*pi;
disp('random q');disp(robot.q);
figure(1);hold on;axis([-5 5 -5 5]);axis('square');
plotarm(robot);pause(2)
% forward kinematics
robot=nlinkfwdkin(robot);
% inverse kinematics (2 solutions)
qsol=threelink_invkin_subproblem(robot)
% 
qsol1=threelink_invkin_geometric(robot)
%
robot.q=qsol(:,1);plotarm(robot);
robot.q=qsol(:,2);plotarm(robot);
% 
N=100;alpha=.1;
robot.q=(rand(size(robot.q))-.5)*2*.05;
robot=invkin_iterJ(robot,100,.1);
qsol2=robot.q
