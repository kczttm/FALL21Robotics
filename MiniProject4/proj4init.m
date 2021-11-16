%
% mini-project 4 initialization of variables
% 
clear all; close all;

zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

% define ABB IRB 1200 robot

L1=399.1;
L2=448; %L2=350;
L3=42;
L4=451; %L4=351;
L5=82;

% P
p01=0*ex+L1*ez;p12=zz;p23=L2*ez;p34=L3*ez+L4*ex;p45=zz;p56=zz;p6T=L5*ex;

% H
h1=ez;h2=ey;h3=ey;h4=ex;h5=ey;h6=ex;

% 
irb1200.P=[p01 p12 p23 p34 p45 p56 p6T];
irb1200.H=[h1 h2 h3 h4 h5 h6];
irb1200.joint_type=[0 0 0 0 0 0];

% % define collision body for abb 1200
radius=.01;
[irb1200_rbt,colLink]=defineRobot(irb1200,radius);

% S-shaped curve

load proj3example/S_sphere_path_uniform l pS
% 
% find the end effector frame
r=.5;
pc=r*ez;
N=length(pS);
xT=zeros(3,N);zT=zeros(3,N);yT=zeros(3,N);
quat=zeros(4,N); % allocate space for unit quaternion representation of R_{0T}

for i=1:N   
    xT(:,i)=(pS(:,i)-pc)/norm(pS(:,i)-pc);
    if i<N
        yT(:,i)=(pS(:,i+1)-pS(:,i));
    else
        yT(:,i)=yT(:,i-1);
    end
    yT(:,i)=yT(:,i)-yT(:,i)'*xT(:,i)*xT(:,i);
    yT(:,i)=yT(:,i)/norm(yT(:,i));
    zT(:,i)=cross(xT(:,i),yT(:,i));
    R=[xT(:,i) yT(:,i) zT(:,i)];
    quat(:,i)=R2q(R);
end
% 
%
% R2q.m
%
% converts R in SO(3) to unit quaternion q, (q0,q_vec)
%
function q=R2q(R)
  
  q=zeros(4,1);
  q(1)=.5*sqrt(trace(R)+1);
  if abs(q(1))<1e-5
    [k,theta]=R2kth(R);
    q(2:4)=k;
  else
    q(2:4)=vee(R-R')/4/q(1);
  end
end

%
% hat.m (converting a vector into a skew-symmetric cross-product matrix
%
% khat = hat(k)
%

function khat = hat(k)
  
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];

end

%
% rot.m
%
% rotation matrix given axis and angle
%
function R=rot(k,theta)
  
  k=k/norm(k);
  R=eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);

end
