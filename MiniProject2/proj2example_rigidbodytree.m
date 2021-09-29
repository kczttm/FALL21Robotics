%
% initialization
%
clear all;close all;

%
% define unit vectors
%
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

%
% load the letter S as a curve
%

load S_letter_curve

figure(1);plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
xlabel('x');ylabel('y');
axis([0 3 -1.5 1.5]);axis('square');grid;
%
[xT,yT]=setR0T(Sls); % <<<< you need to provide this

% robot parameters
l1 = 1.5; l2 = 1.5; l3 = 0.5;
%l1 = 1.5; l2 = 1.5; l3 = 0.1;
%l1 = 2; l2 = 1.5; l3 = 0.25;
%l1 = 1.5; l2 = 1.5; l3 = 1;
robot.P = [zz l1*ex l2*ex l3*ex];
robot.H = [ez ez ez];
robot.joint_type=[0 0 0];

% zero configuration 

robot.q=[0;0;0];
% radius of the link (as cylinders)
radius = .01;
robot_rb=defineRobot(robot,radius);
figure(10);hold on;show(robot_rb,robot.q,'Collision','on');
view(0,90);
axis([0,4,-2,2]);axis('square');
xlabel('x-axis');ylabel('y-axis');
title('Planar RRR arm in zero configuration (q_1=q_2=q_3=0)')
hold off

%
nl=length(Sls);
qsol1=zeros(3,nl-1);
qsol2=zeros(3,nl-1);
for i=1:nl-1
    robot.T(1:3,1:4)=[xT(:,i) yT(:,i) ez [Sls(:,i);0]];
    qsol=threelink_invkin_geometric(robot);% <<<< you need to provide this
    qsol1(:,i)=qsol(:,1);
    qsol2(:,i)=qsol(:,2);    
end

figure(2);plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
hold on
grid;axis([-1,3,-2,2]);axis('square');
for i=1:3:nl-1
    robot.q=qsol1(:,i);pause(.1);
    show(robot_rb,robot.q,'Collision','on'); 
    view(0,90);
end
grid;
fprintf('max joint speed: %5.4f, %5.4f, %5.4f\n',max(abs(diff(qsol1')')')');

figure(3);plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
hold on
axis([-1,3,-2,2]);axis('square');
for i=1:3:nl-1
    robot.q=qsol2(:,i);
    show(robot_rb,robot.q,'Collision','on'); 
    view(0,90);
end
grid;
fprintf('max joint speed: %5.4f, %5.4f, %5.4f\n',max(abs(diff(qsol2')')')');

%************** function *******************
%
% 3x3 skew-symmetric matrix
%
function vhat=crossmat(v)

    vhat = [0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];

end
