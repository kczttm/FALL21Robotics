% ABB IRB 1200 parameters
L1=399.1;
L2=448;
L3=42;
L4=451;
L5=82;
% P: positon vector of each link
p01=0*ex+L1*ez;
p12=zz;
p23=L2*ez;
p34=L3*ez+L4*ex;
p45=zz;
p56=zz;
p6T=L5*ex;
% H: rotation axis
h1=ez;
h2=ey;
h3=ey;
h4=ex;
h5=ey;
h6=ex;
% IRB 1200 robot using POE convention
irb1200.P=[p01 p12 p23 p34 p45 p56 p6T]/1000; % unit in meter
irb1200.H=[h1 h2 h3 h4 h5 h6];
irb1200.joint_type=[0 0 0 0 0 0]; % all rotational joint
radius=.01;
[irb1200_rbt,colLink]=defineRobot(irb1200,radius);