%
% Part 3 of MiniProject 5
%
% hand-eye calibration example using the soution of the general pnp problem
% 
clear all;close all;

% define a pin hole camera
cam_def

% load S-shape curve in reference frame
%load S_sphere_path.mat
load S_sphere_path_uniform.mat

% define ABB IRB 1200 robot

zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];
L1=399.1/1000;L2=448/1000;L3=42/1000;L4=451/1000;L5=82/1000;
% P
p01=0*ex+L1*ez;p12=zz;p23=L2*ez;p34=L3*ez+L4*ex;p45=zz;p56=zz;p6T=L5*ex;
% H
h1=ez;h2=ey;h3=ey;h4=ex;h5=ey;h6=ex;
% 
irb1200.P=[p01 p12 p23 p34 p45 p56 p6T];
irb1200.H=[h1 h2 h3 h4 h5 h6];
irb1200.joint_type=[0 0 0 0 0 0];

% **** These are the paraemters to estimate **** 
% translation of the robot base from the reference frame of the S-shape
% (assuming same orientation) represented in the reference frame
P_0B=[2.5;0;0];
% translation of the camera frame origin from end effector frame (assuming
% same orientation) represented in the end effector frame
P_TC=[0;0;0.05];
% ****
T_TC=[[ey ez ex],P_TC ;[0 0 0 1]];
T_0B=[eye(3,3) P_0B;[0 0 0 1]];

% for hand-eye calibration, generate M images by moving robot around 
M = 4;
q = cell(1,M);
for i=1:M
    full_image_in_frame_flag = 0;

    % keep on moving robot randomly until full image is captured in the frame

    while full_image_in_frame_flag == 0
        %irb1200.q=[0;-.2;2.6;pi;-.7;0]+[.01;.01;.01;.01;.01;.01]*50.*(rand(6,1)-.5);
        irb1200.q=[0;1.6;3;3;2;0]+[.01;.01;.01;.01;.01;.01]*50.*(rand(6,1)-.5);
        %irb1200=fwddiffkiniter(irb1200);
        irb1200=nlinkfwdkin(irb1200);
        % calculate camera frame and acquire image 
        T_0C=T_0B*irb1200.T*T_TC;
        [uv{i},uvw,P1]=cam_image(cam,T_0C,pS);
        uv{i}=uv{i}+randn(size(uv{i}))*cam.ns;
        %disp(size(uv{i},2));
        % check if image is completely in frame, if not, continue
        if size(uv{i},2) == size(pS,2)
            full_image_in_frame_flag = 1;
        end
    end
    % save forward kinematics
    q{i} = irb1200.q;
    T_BT{i}=irb1200.T;
    % *** This is the hand-eye calibration for Part 3
    [T_C0_est,Zest]=pnp_general(uv{i},pS,cam.K);
    T_0C_est{i}=inv(T_C0_est);
end

% show camera image 
for i=1:M 
    figure(40);plot(uv{i}(1,:),uv{i}(2,:),'x','linewidth',2); hold on;
    xlabel('image plane horizontal axis');
    ylabel('image plane vertical axis'); 
    p0C=T_0C(1:3,4);
    title(['camera frame location: ',...
    '[',num2str(p0C(1)),',',num2str(p0C(2)),',',num2str(p0C(3)),']']);
    axis([0 cam.uv0(1)*2 0 cam.uv0(2)*2]);
    set ( gca, 'xdir', 'reverse' )
end
hold off;

% estimate the unknown vectors p_{TC} and p_{0B}

A=zeros(3*M,6);
b=zeros(3*M,1);
for i=1:M
    b(3*(i-1)+1:3*i)=T_0C_est{i}(1:3,4)-T_BT{i}(1:3,4);
    A(3*(i-1)+1:3*i,1:3)=T_BT{i}(1:3,1:3);
    A(3*(i-1)+1:3*i,4:6)=eye(3,3);
end
x = pinv(A)*b;
P_TC_est=x(1:3);
P_0B_est=x(4:6);

T_TC_est=[[ey ez ex],P_TC_est ;[0 0 0 1]];
T_0B_est=[eye(3,3) P_0B_est;[0 0 0 1]];

disp('p_TC vs. estimate');
disp([P_TC P_TC_est]);

disp('p_0B vs. estimate');
disp([P_0B P_0B_est]);
