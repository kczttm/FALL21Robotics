%
% finding a rough free space for a 3 link arm
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

% generate collision body
S_radius=.01;
S_col=collision_S(Sls,S_radius);

% 3-link robot parameters
n=3;l=[1.5 1.5 .5];
robot.P = zeros(3,n+1);
robot.P(1,2:4) = l;
robot.H = zeros(3,n);
robot.H(3,:)=ones(1,n);
robot.joint_type=zeros(1,n);

% generate robot collision body
radius = .01;
[robot_rb,colLink]=defineRobot(robot,radius);

% check collision for a grid of (q1,q2,q3)

ngrid=[10 10 6];qlim=[pi/2 4*pi/3 4*pi/3];
qmin=-qlim;qmax=qlim;dq=(qmax-qmin)./ngrid;
% q1min=-qlim(1);q1max=qlim(1);dq1=(q1max-q1min)/ngrid(1);
% q2min=-qlim(2);q2max=qlim(2);dq2=(q2max-q2min)/ngrid(2);
% q3min=-qlim(3);q3max=qlim(3);dq3=(q3max-q3min)/ngrid(3);

q=(qmin:dq:qmax);
q1=(q1min:dq1:q1max);
q2=(q2min:dq2:q2max);
q3=(q3min:dq3:q3max);

nq1=numel(q1);
nq2=numel(q2);
nq3=numel(q3);

colgrid=zeros(nq1,nq2,nq3);

%
% generation of free space
%

fignum=10;
for i=1:nq1
    for j=1:nq2
        for k=1:nq3
            robot.q=[q1(i);q2(j);q3(k)];
            int=robot_Scol_dist(robot,colLink,S_col);
            colgrid(i,j,k)=max(int);
            disp([i j k colgrid(i,j,k)]);
        end
    end
end

for i=1:length(q3)
    [i1{i},i2{i}]=find(colgrid(:,:,i)>0);
    figure(20);plot3(q1(i1{i}),q2(i2{i}),q3(i)*ones(length(i1{i}),1),'x');
    hold on;
end




%************** function *******************
%
% 3x3 skew-symmetric matrix
%
function vhat=crossmat(v)

    vhat = [0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];

end

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
function [d,indrobot,indS,p]=robot_S_dist(robot,S)
    p=pjoint(robot);np=size(p,2);nS=size(S,2);
    pS=kron(ones(nS,1),p)-kron(ones(1,np),reshape(S,2*nS,1));
    pSvec=reshape(pS,2*nS*np,1);
    pSmat=reshape(pSvec,2,nS*np);
    [d,ind]=min(vecnorm(pSmat));
    indS=mod(ind,nS)+1;
    indrobot=fix(ind/nS);
end

%
% robot_Scol_dist.m
%
function isIntS=robot_Scol_dist(robot,colLink,S)

for i=1:length(S)
    [isInt,dist,wp]=RobotCollisionCheck(robot,colLink,S{i});
    isIntS(i)=max(cell2mat(isInt));
end

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

    s=(hI>eta).*(-M*atan(c*(hI-eta)))+(hI>=0).*(e*(eta-hI)/eta)+(hI<0).*e;

end

