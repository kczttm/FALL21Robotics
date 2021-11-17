%
% PBVS_unlimited_range.m
%
% PBVS example using damped least square.  If image is out of range, then
% use robot forward kinematics and calibration parameters.
%
% input:
%       robot structure with 
%       robot.T as the target orientation and position
%       robot.q as the initial guess
%       robot.MaxIter = max # of iterations
%       robot.StepSize = update step size
%       robot.Weights = weighting for end effector error
%       
%       cam: pin-hole camera object
%
%       pS: target coordinate in 0 frame
%
%       T_0C_des: target camera frame 
%
%       T_0B: transform from 0 to robot base frame B
%       T_TC: transform from robot task frame to camera frame
%       T_0B_est: estimated T_0B from hand-eye calibration
%       T_TC_est: estimated T_TC from hand-eye calibration
%
function [q,uv,T_0C]=PBVS_unlimited_range(robot,cam,pS,T_0C_des,...
    T_0B,T_TC,T_0B_est,T_TC_est)
        
    % set up parameters
    
    N = robot.MaxIter;
    alpha = robot.StepSize;
    weights = robot.Weights; 

    % convert desired camera pose to desired robot pose using parameters
    % from hand-eye calibration (hand_eye_cal.m)
    T_BT_des=inv(T_0B_est)*T_0C_des*inv(T_TC_est);

    % set up storage space
    q0=robot.q;
    n=length(q0); % # of joints
    q=zeros(n,N+1); 
    q(:,1)=q0; % output joint displacements
    %p0T=zeros(3,N+1); % output p
    %quatT=zeros(4,N+1); % output quaternion
    
    % iterative update (N loops)
    for i=1:N
        % forward kinematics
        robot.q=q(:,i);
        %robot=fwddiffkiniter(robot);
        robot=nlinkfwdkin(robot);
        % generate a camera image
        T_0C{i}=T_0B*robot.T*T_TC;
        % generate image (same as in Part 1 of miniproject 5)
        [uv{i},uvw,P1]=cam_image(cam,T_0C{i},pS);
        % add some noise
        uv{i}=uv{i}+randn(size(uv{i}))*cam.ns;
        % check if image is within frame
        if size(uv{i},2)==size(pS,2)
            % if yes, use the camera image to estimate T_C0 by solving pnp problem        
            [T_C0_est,Zest]=pnp_general(uv{i},pS,cam.K);
            T_0C_est=inv(T_C0_est);
        else
            % if not, use robot and calibration parameter to estimate T_0C
            T_0C_est=T_0B_est*robot.T*T_TC_est;
            disp('image out of range');
        end
        T_BT_est=inv(T_0B_est)*T_0C_est*inv(T_TC_est);
        % task space error 
        dX=[R2qv(T_BT_est(1:3,1:3)*T_BT_des(1:3,1:3)');...
            T_BT_est(1:3,4)-T_BT_des(1:3,4)];
        % damped least square control
        qq=q(:,i)-alpha*robot.J'*inv(robot.J*robot.J'+.01*diag(1./weights))*dX;
        q(:,i+1)=(qq>pi).*(-2*pi+qq)+(qq<-pi).*(2*pi+qq)+(qq<pi).*(qq>-pi).*qq;
    end
    % q from the final iteration
    robot.q=q(:,N+1);
    % compute (RBT,pBT) for q from final iteration
    robot=nlinkfwdkin(robot);
    T_0C{N+1}=T_0B*robot.T*T_TC;
end

%
% R2qv.m
%
% converts R in SO(3) to vector quaternion q_vec
%

function qv=R2qv(R)
  
  q0=.5*sqrt(trace(R)+1);
  if abs(q0)<1e-5
    [k,theta]=R2kth(R);
    qv=k;
  else
    qv=vee(R-R')/4/q0;    
  end
  
end

