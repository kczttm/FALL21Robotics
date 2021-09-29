%
% kinematic control with collision avoidance using 
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

S_radius=.01;
S_col=collision_S(Sls,S_radius);

[xT,yT]=setR0T(Sls); % <<<< you need to provide this

% robot parameters
n=80;l=4/n;
%n=50;l=5/n;
robot.P = zeros(3,n+1);
for i=1:n;robot.P(:,i+1)=l*[1;0;0];end
robot.H = zeros(3,n);
robot.H(3,:)=ones(1,n);
robot.joint_type=zeros(1,n);

% truncated robot for collision calculation
robcol=robot;ncol=fix(.7*n);
robcol.P=robcol.P(:,1:ncol+1);
robcol.H=robcol.H(:,1:ncol);
robcol.joint_type=robcol.joint_type(1:ncol);

% set path speed and time vaector

ldot = .1; % uniform \dot\lambda
t=ls/ldot; % time vector
N=length(t);
% 
Nmax=fix(1.2*N-1);
dt=mean(diff(t));
t=(0:dt:(Nmax-1)*dt);
q=zeros(n,Nmax);
% initial configuration
q0=[-pi;-pi/n*ones(n-1,1)];
q(:,1)=q0;
% proportional feedback gain
Kp=diag(ones(n,1))*.8;
% joint velocity limit
umax=ones(n,1);

% for collision avoidance (sigmafun)
e=.025;eta=0.04;c=100;M=100;
hI=(-1:.05:3);
s=sigmafun(hI,eta,c,M,e);
figure(5);plot(hI,s,'-','linewidth',2);grid
xlabel('hI');ylabel('\sigma');title('control barrier function');
%

% initial portion is to get to the starting point
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
    robot=nlinkfwdkin(robot);
    qT(:,k)=atan2(robot.T(2,1),robot.T(1,1));
    pT(:,k)=robot.T(1:2,4);
    % form task space error 
    dX=[qT(:,k)-qTd;pT(:,k)-pTd];
    %
    % for collision avoidance
    %
    % define a shorter robot for collision checking
    %
    robcol.q=robot.q(1:ncol);
    % just use robot joint locations and grid points of S to check distance
    [d,indrobot,indS,p]=robot_S_dist(robcol,Sls);
    % default inequality constraint is null
    A=[];b=[];
    % if distance is less too close, then add an inquality constraint
    if d<eta
        % first find vectors at closest points (robot and S)        
        wpvec=Sls(:,indS)-p(:,indrobot);
        % inequality constraint is the closest distance squared 
        % d^2=(wpvec'*wpvec)
        hI=d^2;
        % define a truncated robot up to the closest distance
        rob_d.P=robcol.P(:,1:indrobot);
        rob_d.H=robcol.H(:,1:indrobot-1);
        rob_d.joint_type=robcol.joint_type(1:indrobot-1);
        rob_d.q=robcol.q(1:indrobot-1);
        % find the Jacobian which is Jd=-\partial wpvec/\partial q
        robcol=nlinkfwdkin(robcol);
        J_d=robcol.J;
        % hI' = -wpvec'* Jd (position portiona only)
        % form inequality constraint
        Acol=-wpvec'*J_d(2:3,:);bcol=-sigmafun(hI,eta,c,M,e);
        % only use joints upto the closest point
        A=[Acol zeros(1,n-ncol)];b=bcol;
    end
    % Jacobian kinematics control using quadratic programming (QP)
    opt = optimset('display','off');
    u(:,k)=quadprog(robot.J'*robot.J,Kp*robot.J'*dX,...
        A,b,[],[],-umax,umax,zeros(n,1),opt);    
    % implement the control law
    qq=q(:,k)+(t(k+1)-t(k))*u(:,k);
    q(:,k+1)=(qq>pi).*(-2*pi+qq)+(qq<-pi).*(2*pi+qq)+(qq<pi).*(qq>-pi).*qq;
end

% plot and save as movie
j=1;
for k=1:3:Nmax
    h=figure(10);
    plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);hold on;
    robot.q=q(:,k);
    plot2drobot(robot);
    view(0,90);axis([-1,3,-2,2]);axis('square');
    drawnow;Mrobot(j)=getframe;j=j+1;
    hold off
end
grid;
fprintf('max joint speed: %5.4f, %5.4f, %5.4f\n',max(abs(diff(q')')')');

%************** function *******************
%
% collsion_S
%

function S_colbody=collision_S(S,radius)

for i=1:size(S,2)-1
    S_colbody{i} = collisionCylinder(radius,.1);
    dS=S(:,i+1)-S(:,i);
    y=dS/norm(dS);
    x=cross([0;0;1],[y;0]);
    S_colbody{i}.Pose(1:3,1:3)=rot(x,pi/2);
    S_colbody{i}.Pose(1:2,4)=dS/2+S(:,i);
end

end

%
% showS.m
%
% show S as a collection of collision bodies
%
function showS(S_colbody)
    for i=1:length(S_colbody);show(S_colbody{i});end;
end

%
% rot.m
% 
% rotation about a given vector
%
function R=rot(k,theta)
  
  k=k/norm(k);
  R=eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
  
end

%
% hat.m
% 
% cross product matrix
%
function khat = hat(k)
  
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
  
end

%
% pjoint.m
%
% calculate coordinae of each joint of a robot struct
%
function p=pjoint(robot)

    q=robot.q;n=length(q);
    
    p=zeros(2,n+1);R=eye(2,2);
    p(:,1)=robot.P(1:2,1);%+rot2(q(1))*robot.P(:,2);
    for i=1:n
        R=rot2(q(i))*R;
        p(:,i+1)=R*robot.P(1:2,i+1)+p(:,i);
    end

end

%
% plot2drobot.m
%
function plot2drobot(robot)

    p=pjoint(robot);
    plot(p(1,:),p(2,:),'^-b','linewidth',3);
    
end

%
% robot_S_dist.m
%
% distance between robot joints and S grid points
%
function [d,indrobot,indS,p]=robot_S_dist(robot,S)
    % robot joints
    p=pjoint(robot);np=size(p,2);nS=size(S,2);
    % form all pairs between robot joints and S grid points
    pS=kron(ones(nS,1),p)-kron(ones(1,np),reshape(S,2*nS,1));
    % convert into one big vector
    pSvec=reshape(pS,2*nS*np,1);
    % convert into a matrix with 2 rows
    pSmat=reshape(pSvec,2,nS*np);
    % find minimum distance
    [d,ind]=min(vecnorm(pSmat));
    % find corresponding indices
    indS=mod(ind,nS)+1;
    indrobot=fix(ind/nS);
end

%
% sigmafun.m: control barrier function to keep constraint function hI>0. If
% 0<hI<eta, hI is a linear function decreasing from e down to 0. If hI>eta,
% hI = -M atan(c(hI-eta)) which rapidly approaches -infinity
% 
% input: 
%       hI = value of the constraint function
%       eta = threshold within which to push the robot away
%       c = controlling the rate of descrease of sigma 
%       M = controlling where sigma converges to (-M)
%       e = strength of repelling 
%
function s=sigmafun(hI,eta,c,M,e)

    s=(hI>eta).*(-M*atan(c*(hI-eta))*2/pi)+...
        (hI>=0).*(hI<eta).*(e*(eta-hI)/eta)+(hI<0).*e;

end

%
% rot2.m
%
function R=rot2(q)
    c=cos(q);s=sin(q);R=[c -s;s c];
end


