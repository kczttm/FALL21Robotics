%
% IBVS_unlimited_range.m
%
% IBVS example using damped least square.  When target is not in view, use
% robot forward kinematics.
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
function [q,uv,T_0C,JI,J,erruv]=IBVS_unlimited_range(robot,cam,pS,uv_des,...
    T_0B,T_TC,T_0B_est,T_TC_est)
        
    % set up parameters
    
    N = robot.MaxIter;
    alpha = robot.StepSize;
    
    P_TC=T_TC(1:3,4);
    P_TC_est=T_TC_est(1:3,4);
    
    % convert desired camera pose to desired robot pose using parameters
    % from hand-eye calibration (hand_eye_cal.m)
    % T_BT_des=inv(T_0B_est)*T_0C_des*inv(T_TC_est);

    % set up storage space
    q0=robot.q;
    n=length(q0); % # of joints
    q=zeros(n,N+1); 
    q(:,1)=q0; % output joint displacements
    %p0T=zeros(3,N+1); % output p
    %quatT=zeros(4,N+1); % output quaternion
    
    % iterative update (N loops)
    for i=1:N
        image_frame_flag = 0;
        while image_frame_flag == 0
            % forward kinematics
            robot.q=q(:,i);
            %robot=fwddiffkiniter(robot);
            robot = nlinkfwdkin(robot);
            % robot Jacobian
            J{i}=robot.J;
 
            % generate a camera image
            T_0C{i}=T_0B*robot.T*T_TC;
            T_C0=inv(T_0C{i});          
            % generate image (same as in Part 1 of miniproject 5)
            [uv{i},uvw,P1]=cam_image(cam,T_0C{i},pS);
            if size(uv{i},2)<size(pS,2);disp('image out of range');end
            %[uv{i},uvw,P1]=cam_image_unlimited_range(cam,T_0C{i},pS);
            % add some noise
            uv{i}=uv{i}+randn(size(uv{i}))*cam.ns;
            % check if full image of S is within frame
            if size(uv{i},2)==size(pS,2)
                image_frame_flag = 1;
            else
                q(:,i)=q(:,i).*(1+randn(6,1)*.01);
            end
        end    
        % compute image Jacobian
        
        % estimated T_0C
        T_0C_est{i}=T_0B_est*robot.T*T_TC_est;
        T_C0_est=inv(T_0C_est{i});   
        % first just to the target frame
        Jc=phi(-T_C0_est(1:3,1:3),T_C0_est(1:3,1:3)'*T_C0_est(1:3,4))*...
            phi(eye(3,3),P_TC_est)*J{i};
        % then from target frame to each point on the target
        for j=1:size(pS,2)
            wj_est=[0 0 1]*cam.K*(T_C0_est(1:3,1:3)*pS(:,j)+T_C0_est(1:3,4));
            JI{i}(2*(j-1)+1:2*j,:)=...
                [eye(2,2) -uv{i}(:,j)]*cam.K*...
                [-hat(T_C0_est(1:3,1:3)*pS(:,j)) eye(3,3)]*Jc/wj_est;
%                [-hat(T_C0(1:3,1:3)*pS(:,j)) eye(3,3)]*Jc/uvw(3,j);
        end        
        % damped least square control using image Jacobian
        np=size(JI{i},1);
        % adjust gain based on closeness to singularity
        alpha1=alpha;
        if min(svd(J{i}))<.1;alpha1=alpha/2;end
        % larger epsilon reduces the sensitivity to the Jacobian
        % singularity
        epsilon = .1;
        % 
        qq=q(:,i)-alpha1*JI{i}'*inv(JI{i}*JI{i}'+epsilon*eye(np,np))*...
            reshape((uv{i}-uv_des),2*size(pS,2),1);
        q(:,i+1)=(qq>pi).*(-2*pi+qq)+(qq<-pi).*(2*pi+qq)+(qq<pi).*(qq>-pi).*qq;
        erruv(:,i)=reshape((uv{i}-uv_des),2*size(pS,2),1);
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

