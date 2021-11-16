%
% mini-project 4 example script
% 
%clear all; close all;

zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

% symbolic ABB IRB 1200 robot

syms L1 L2 L3 L4 L5 positive
syms q1 q2 q3 q4 q5 q6 real

% define ABB IRB 1200 robot symbolically

% P
p01=0*ex+L1*ez;p12=zz;p23=L2*ez;p34=L3*ez+L4*ex;p45=zz;p56=zz;p6T=L5*ex;
%p01=0*ex+L1*ez;p12=zz;p23=L2*ez;p34=L4*ex;p45=zz;p56=zz;p6T=L5*ex;

% H
h1=ez;h2=ey;h3=ey;h4=ex;h5=ey;h6=ex;

% q
q=[q1;q2;q3;q4;q5;q6];

% forward kinematics and Jacobian

irb1200_s.P=[p01 p12 p23 p34 p45 p56 p6T];
irb1200_s.H=[h1 h2 h3 h4 h5 h6];
irb1200_s.joint_type=[0 0 0 0 0 0];
irb1200_s.q=q;

irb1200_s=fwddiffkiniter(irb1200_s);

% analytical Jacobian in base frame

%JT_0=simplify(irb1200_s.J);
JT_0=irb1200_s.J;
R0T=simplify(irb1200_s.T(1:3,1:3));
% p0T=simplify(irb1200_s.T(1:3,4));
% J4_0=phi(eye(3,3),-R0T*p6T)*JT_0;
% R02=rot(h1,q1)*rot(h2,q2);
% J4_2=[R02' zeros(3,3);zeros(3,3) R02']*J4_0;
% J4_2(4:6,4:6)=simplify(J4_2(4:6,4:6));
% J4_2(1:3,4:6)=simplify(J4_2(1:3,4:6));
% J4_2(4:6,1:3)=simplify(J4_2(4:6,1:3));
% J4_2(1:3,1:3)=simplify(J4_2(1:3,1:3));
% pretty(J4_2)
% R23=rot(h3,q3);
% J4_3=simplify([R23' zeros(3,3);zeros(3,3) R23']*J4_2);
R03=rot(h1,q1)*rot(h2,q2)*rot(h3,q3);
J4_3=simplify(phi(R03',-R0T*p6T)*JT_0);
%JT_0=phi(R03,R03'*R0T*p6T)*J4_3;
pretty(J4_3)

%save IRB1200Jacobian JT_0 J4_2 J4_3

%
% phi.m
% 
% propagation of spatial velocity
%
function phimat=phi(R,p)

    phimat=[R zeros(3,3);-R*hat(p) R];
    
end

%
% hat.m (converting a vector into a skew-symmetric cross-product matrix
%
% khat = hat(k)
%

function khat = hat(k)
  
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];

end
