%
% Kinematic Control for an n-link arm to follow the S-shape
%

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
% specify end effector orientation
[xT,yT]=setR0T(Sls); % <<<< you need to provide this

% robot parameters
%n=50;l=3.5/n;
n=100;l=4/n;
robot.P = zeros(3,n+1);
for i=1:n;robot.P(:,i+1)=l*[1;0;0];end
robot.H = zeros(3,n);
robot.H(3,:)=ones(1,n);
robot.joint_type=zeros(1,n);

% radius of the link (as cylinders)
radius = .01;
robot_rb=defineRobot(robot,radius);
% 

% specify a uniform path speed \dot\lambda
ldot = .3; 
t=ls/ldot; % time vector
N=length(t);
% 

Nmax=fix(1.5*N-1);
dt=mean(diff(t));
t=(0:dt:(Nmax-1)*dt);
q=zeros(n,Nmax);
% initial arm configuration
q0=[-pi;-pi/n*ones(n-1,1)];
q(:,1)=q0;
% proportional feedback gain
Kp=diag(ones(n,1));
% weighting in damped least square to avoid excessive speed
epsilon=.1;
% maximum joint speed
umax=ones(n,1);

% allow sound time to reach the initial starting point of the S-shape
% and add some time after the final point in the S-shape
N1=fix(N/10);
for k=1:Nmax-1
    % desired target
    if k<N1
        qTd=atan2(xT(2,1),xT(1,1));
        pTd=Sls(:,1);
    elseif k<N+N1-1
        qTd=atan2(xT(2,fix(k-N1+1)),xT(1,fix(k-N1+1)));
        pTd=Sls(:,fix(k-N1+1));      
    else 
        qTd=atan2(xT(2,N-1),xT(1,N-1));
        pTd=Sls(:,N);         
    end
    robot.q=q(:,k);
    % forward kinematics to find current pose and Jacobian
    robot=nlinkfwdkin(robot);
    qT(:,k)=atan2(robot.T(2,1),robot.T(1,1));
    pT(:,k)=robot.T(1:2,4);
    % form task space error 
    dX=[qT(:,k)-qTd;pT(:,k)-pTd];
    % Jacobian kinematics control (soft constraint on qdot)
    u(:,k)=-Kp*robot.J'*inv(robot.J*robot.J'+epsilon*eye(3,3))*dX;
    % constraint the maximum speed
    u(:,k)=(u(:,k)>umax).*umax+(u(:,k)<-umax).*(-umax)+...
        (u(:,k)<umax).*(u(:,k)>-umax).*u(:,k);
    % update robot motion using qdot (based on finite difference)
    qq=q(:,k)+(t(k+1)-t(k))*u(:,k);
    % restrict to be in -pi to pi
    q(:,k+1)=(qq>pi).*(-2*pi+qq)+(qq<-pi).*(2*pi+qq)+(qq<pi).*(qq>-pi).*qq;
end

% show robot motion 
for k=1:3:Nmax
    h=figure(10);
    plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);hold on;
    show(robot_rb,q(:,k),'Collision','on'); 
    view(0,90);axis([-1,3,-2,2]);axis('square');
    % pause(.1);
    hold off
end
grid;
fprintf('max joint speed: %5.4f, %5.4f, %5.4f\n',max(abs(diff(q')')')');

%************** function *******************
%
% 3x3 skew-symmetric matrix
%
% function vhat=crossmat(v)
% 
%     vhat = [0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
% 
% end
