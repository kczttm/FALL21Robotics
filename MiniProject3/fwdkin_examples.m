%
% forward kinematics examples
%

zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

syms q1 q2 q3 q4 q5 q6 real
q=[q1;q2;q3;q4;q5;q6];

syms l0 l1 l2 l3 l4 real

% elbow

elbow.H=[ez,ey,ey,ey,ez,ex];
elbow.P=[l0*ez,zz,l1*ex,l2*ex,zz,zz,l3*ex];
elbow.joint_type=[0 0 0 0 0 0];
elbow.q=q;
elbow=fwdkiniter(elbow);
%elbow=fwddiffkiniter(elbow);

% SCARA

h1=ez;h2=ez;h3=ez;h4=ez;
p01=zz;p12=l1*ey;p23=l2*ey;p34=l0*ez;p4T=zz;
scara.H=[h1 h2 h3 h4];
scara.P=[p01 p12 p23 p34 p4T];
scara.joint_type=[0 0 0 1];
scara.q=q(1:4);
scara=fwdkiniter(scara);

% Rhino

h1=ex;h2=ez;h3=ey;h4=ey;h5=ey;h6=ex;R6T=eye(3,3);
p01=l0*ex;p12=l1*ez;p23=zz;p34=l2*ex;p45=l3*ex;p56=zz;p6T=l4*ex;
rhino.H=[h1 h2 h3 h4 h5 h6];
rhino.P=[p01 p12 p23 p34 p45 p56 p6T];
rhino.joint_type=[1 0 0 0 0 0];
rhino.q=q;
rhino=fwdkiniter(rhino);

% Stanford

h1=ez;h2=ex;h3=ey;h4=ey;h5=ez;h6=ey;R6T=eye(3,3);
p01=l0*ez;p12=l1*ex;p23=l3*ey;p34=zz;p45=zz;p56=zz;p6T=l4*ey;
stanford.H=[h1 h2 h3 h4 h5 h6];
stanford.P=[p01 p12 p23 p34 p45 p56 p6T];
stanford.joint_type=[0;0;0;0;0;0];
stanford.q=q;
stanford=fwdkiniter(stanford);

% inverse elbow

h1=ez;h2=ey;h3=ex;h4=ey;h5=ey;h6=ex;
p01=zz;p12=zz;p23=zz;p34=l1*ex;p45=l2*ex;p56=zz;p6T=l3*ex;
inverse_elbow.H=[h1 h2 h3 h4 h5 h6];
inverse_elbow.P=[p01 p12 p23 p34 p45 p56 p6T];
inverse_elbow.joint_type=[0;0;1;0;0;0];
inverse_elbow.q=q;
inverse_elbow=fwdkiniter(inverse_elbow);

% cross production function
function khat = hat(k)
  
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
  
end