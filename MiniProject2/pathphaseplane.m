%
% find lambdadot max for each lambda
%

%
% initialization
%
%clear all;close all;
addpath(genpath('C:\Users\kongc\Dropbox\RPI_2021FALL\RoboticsI\MATLABrelated\'))
%
% define unit vectors
%
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

%
% load the letter S as a curve
%

load S_letter_path.mat

figure(1);plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
xlabel('x');ylabel('y');
axis([0 3 -1.5 1.5]);axis('square');grid;
%
[xT,yT]=setR0T(Sls); % <<<< you need to provide this
ls = [0, cumsum(vecnorm(diff(Sls')'))];

% robot parameters
l1 = 1.5; l2 = 1.5; l3 = 0.5;
%l1 = 1.5; l2 = 1.5; l3 = 0.1;
%l1 = 2; l2 = 1.5; l3 = 0.25;
%l1 = 1.5; l2 = 1.5; l3 = 1;
robot.P = [zz l1*ex l2*ex l3*ex];
robot.H = [ez ez ez];
robot.joint_type=[0 0 0];

% 
umax=[0.2;0.2;0.1];
amax=[.05;.05;.05];

N=length(ls); % provided ls
qT=zeros(1,N);
pT=zeros(2,N);
qTprime=zeros(1,N);
pTprime=zeros(2,N);
q1=zeros(3,N);
q2=zeros(3,N);

ldot1=zeros(1,N);
ldot2=zeros(1,N);

for i=1:N-1
    qT(i)=atan2(xT(2,i),xT(1,i));
    pT(:,i)=Sls(:,i); 
    qTprime(i)=(qT(i+1)-qT(i))/(ls(i+1)-ls(i));
    pTprime(:,i)=(pT(:,i+1)-pT(:,i))/(ls(i+1)-ls(i));
    A=[qTprime(i);pTprime(:,i)];
    robot.T(1:3,1:4)=[xT(:,i) yT(:,i) ez [Sls(:,i);0]];
    qsol=threelink_invkin_geometric(robot);
    %
    q1(:,i)=qsol(:,1);
    robot.q=q1(:,i);
    robot=nlinkfwdkin(robot);
    options = optimoptions('linprog','Display','none');
    qdot=linprog(-pinv(A)*robot.J(1:3,:),[],[],[],[],-umax,umax,zeros(3,1),options);
    ldot1(:,i)=pinv(A)*robot.J(1:3,:)*qdot;
    %
    q2(:,i)=qsol(:,2);
    robot.q=q2(:,i);
    robot=nlinkfwdkin(robot);
    qdot=linprog(-(pinv(A)*robot.J(1:3,:))',[],[],[],[],-umax,umax,zeros(3,1),options);
    ldot2(:,i)=pinv(A)*robot.J(1:3,:)*qdot;
end

figure(11);plot(ls,ldot1,'linewidth',2);
figure(12);plot(ls,ldot2,'linewidth',2);
