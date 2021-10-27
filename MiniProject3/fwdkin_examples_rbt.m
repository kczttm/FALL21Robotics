%
% forward kinematics examples
%
clearvars
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

% 
q=zeros(6,1);
l0=.2;l1=.1;l2=.3;l3=.3;l4=.1;
radius=.05;

% elbow

elbow.H=[ez,ey,ey,ey,ez,ex];
elbow.P=[zz,zz,l1*ex,l2*ex,zz,zz,l3*ex];
elbow.joint_type=[0 0 0 0 0 0];
elbow.q=q;
elbow=fwdkiniter(elbow);
elbow_rbt=defineRobot(elbow,radius);

% SCARA

h1=ez;h2=ez;h3=ez;h4=ez;
p01=zz;p12=l1*ey;p23=l2*ey;p34=l0*ez;p4T=zz;
scara.H=[h1 h2 h3 h4];
scara.P=[p01 p12 p23 p34 p4T];
scara.joint_type=[0 0 0 1];
scara.q=q(1:4);
scara=fwdkiniter(scara);
scara_rbt=defineRobot(scara,radius);

% Rhino

h1=ex;h2=ez;h3=ey;h4=ey;h5=ey;h6=ex;R6T=eye(3,3);
p01=l0*ex;p12=l1*ez;p23=zz;p34=l2*ex;p45=l3*ex;p56=zz;p6T=l4*ex;
rhino.H=[h1 h2 h3 h4 h5 h6];
rhino.P=[p01 p12 p23 p34 p45 p56 p6T];
rhino.joint_type=[1 0 0 0 0 0];
rhino.q=q;
rhino=fwdkiniter(rhino);
rhino_rbt=defineRobot(rhino,radius);

% Stanford

h1=ez;h2=ex;h3=ey;h4=ey;h5=ez;h6=ey;R6T=eye(3,3);
p01=l0*ez;p12=l1*ex;p23=l3*ey;p34=zz;p45=zz;p56=zz;p6T=l4*ey;
stanford.H=[h1 h2 h3 h4 h5 h6];
stanford.P=[p01 p12 p23 p34 p45 p56 p6T];
stanford.joint_type=[0;0;1;0;0;0];
stanford.q=q;
stanford=fwdkiniter(stanford);
stanford_rbt=defineRobot(stanford,radius);

% inverse elbow

h1=ez;h2=ey;h3=ex;h4=ey;h5=ey;h6=ex;
p01=zz;p12=zz;p23=zz;p34=l1*ex;p45=l2*ex;p56=zz;p6T=l3*ex;
inverse_elbow.H=[h1 h2 h3 h4 h5 h6];
inverse_elbow.P=[p01 p12 p23 p34 p45 p56 p6T];
inverse_elbow.joint_type=[0;0;0;0;0;0];
inverse_elbow.q=q;
inverse_elbow=fwdkiniter(inverse_elbow);
inv_elbow_rbt=defineRobot(inverse_elbow,radius);

%
% defineRobot.m
% Define a robot as a collision body rigid body tree given the
% Product of Expoential (POE) description of a robot arm
%
% input
%       robdef: robot definition structure with
%               robdef.H: motion axis, a 3xn matrix
%               robdef.P: link vector, a 3x(n+1) matrix
%               robdef.joint_type: 1xn vector, 0 for revolute 1 for prismatic
%       rad: radius of the link cylinder (each link assumed to be a
%       cylinder of equal radius).  This is easily changed in the
%       code to have different radii or link shape (e.g., box)
%
% output      
%       robot: MATLAB rigid body tree object
%       colLink: MATLAB link collision body description (in zero
%       configuration) 
%
function [robot,colLink]=defineRobot(robdef,rad)

% product of exponential description
H=robdef.H;
P=robdef.P;
type=robdef.joint_type;
n=numel(type);

% homogeneous transforms T0i stored in T0i{k}, k=1,..,n+1
T=eye(4,4);
for i=1:n
    q(i)=0;
    if type(i)==0
        Ti{i}=[rot(H(:,i),q(i)) P(:,i);[0 0 0 1]];
        T=T*Ti{i};
    else
        Ti{i}=[eye(3,3),P(:,i)+q(i)*H(:,i);[0 0 0 1]];
        T=T*Ti{i};
    end
    T0i{i}=T;
end
Ti{n+1}=[eye(3,3) P(:,n+1);[0 0 0 1]];
T0i{n+1}=T*Ti{n+1};

% define MATLAB rigidbody tree

robot=rigidBodyTree('DataFormat','column');

for i=1:n
    ii=num2str(i);
    eval(['body',ii,' = rigidBody(''','body',ii,''');']);
    if type(i)==0
        eval(['jnt',ii,' = rigidBodyJoint(''','jnt',...
            ii,'''',',''revolute''',');']);
    else
        eval(['jnt',ii,' = rigidBodyJoint(''','jnt',...
            ii,'''',',''prismatic''',');']);
    end
    eval(['jnt',ii,'.JointAxis=H(:,',ii,');']);
end
ii=num2str(n+1);
eval(['body',ii,' = rigidBody(''','body',ii,''');']);
eval(['jnt',ii,' = rigidBodyJoint(''','jnt',...
    ii,'''',',''fixed''',');']);

for i=1:n+1
    ii=num2str(i);
    eval(['setFixedTransform(jnt',ii,',Ti{',ii,'});']);
    eval(['body',ii,'.Joint = jnt',ii,';']);
end

addBody(robot,body1,'base')
for i=2:n+1
    ii=num2str(i);
    iim1=num2str(i-1);
    eval(['addBody(robot,body',ii,',','''body',iim1,''');']);
end

colBodyRadius=rad;

for i=1:n+1
    if norm(P(:,i))>0
        if (i<n+1) && (type(i)>0)
            colLink{i} = collisionBox(colBodyRadius,colBodyRadius,norm(P(:,i)));
        else
            colLink{i} = collisionCylinder(colBodyRadius,norm(P(:,i)));
        end
        kvec=cross([0;0;1],P(:,i));
        if norm(kvec)<sqrt(eps)
            colLink{i}.Pose=trvec2tform(P(:,i)'/2);
        else
            th=subprob0(kvec,[0;0;1],P(:,i));
            colLink{i}.Pose=[rot(kvec,th) P(:,i)/2;[0 0 0 1]];
        end
    else
        colLink{i} = collisionCylinder(colBodyRadius,norm(P(:,i)));
    end
    if i==1
        addCollision(robot.Base,colLink{i});    
    else
        addCollision(robot.Bodies{i-1},colLink{i});    
    end    
end

end

%
% rot.m 
%
% rotation matrix about vector k over angle theta
% 
% R=rot(k,theta)
%

function R=rot(k,theta)
  
  k=k/norm(k);
  R=eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
  
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
% q=subprob0(k,p1,p2)
%
% solve for q subtended between p1 and p2
%    k determines the sign of q
%
% input: k,p1,p2 as R^3 column vectors
% output: q (scalar)
%

function q=subprob0(k,p1,p2)

if ((k'*p1)>sqrt(eps)|(k'*p2)>sqrt(eps))
  error('k must be perpendicular to p and q');
end

p1=p1/norm(p1);
p2=p2/norm(p2);

q=2*atan2(norm(p1-p2),norm(p1+p2));

if k'*(cross(p1,p2))<0
  q=-q;
end

end

