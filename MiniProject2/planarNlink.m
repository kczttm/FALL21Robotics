%
% name: planarNlink.m
% 
% purpose: create an N-link planar robot arm using defineRobot
%
% end result: a robot tree object called "robot"
%
% by: John Wen
% 
% date: 9/18/2021
%

clear all;close all;
% # of links
n=20;

% length of all the links 
l=[0 .3*ones(1,n)];

ex=[1;0;0];ez=[0;0;1];
% all rotation axis are z-axis
robot.H=zeros(3,n);robot.H(3,:)=1;
% all link vectors are l*ex
robot.P=zeros(3,n+1);robot.P(1,:)=l;
%robdef.P(1,:)=[0 1.5 1.5 .5];
% all revolute
robot.joint_type=zeros(1,n);

% define a rigidbody tree
nlinkrobot=defineRobot(robot,.1);

% show the n link robot in a random configuration
q=(rand(n,1)-.5)*2*pi*.1;
figure(20);show(nlinkrobot,q,'collision','on');
view(0,90);
pause(2);

% MATLAB interactive rigid body tree
figure(1);interactiveRigidBodyTree(nlinkrobot);
input('press any key to continue: ');

% click and show

figure(2);show(nlinkrobot,zeros(n,1),'Collision','on');
view(0,90);hold on;

x=0;
while abs(x)<sum(l)*.9
    % cursor input for position
    [x,y]=ginput(1);
    qT=0;
    % qT=atan2(diff(y),diff(x));
    % specify the end point
    T=[cos(qT) -sin(qT) 0 x(1);sin(qT) cos(qT) 0 y(1); 0 0 1 0; 0 0 0 1];
    % define inverse kinematics object
    ik=inverseKinematics('RigidBodyTree',nlinkrobot)
    % initial guess is just the zero configuration
    initialguess = nlinkrobot.homeConfiguration;
    % for the last link
    bodyname=['body',num2str(n+1)];
    % relative weights for the pose error
    weights=[10 10 10 1 1 1];
    % find the solution
    [qsol,solnInfo] = ik(bodyname,T,weights,initialguess);
    % show the solution
    show(nlinkrobot,qsol,'Collision','on');
    T1=getTransform(nlinkrobot,qsol,bodyname);
    disp(norm(T1-T));
    robot.q=(rand(n,1)-.5)*2*0;
    robot=nlinkfwdkin(robot);
    robot.T=T;
    robot=invkin_iterJ(robot,100,.2);
    show(nlinkrobot,robot.q,'Collision','off');
    disp(norm(T-robot.T));
end


