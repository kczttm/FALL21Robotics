%
% invkin_iterJ.m
%
% inverse kinematics using Jacobian iteration (3D)
% use r2q.m

function robot=invkin_iterJ(robot)
    N = robot.MaxIter;
    alpha = robot.StepSize;
    weights = robot.Weights;
    % target (R,p)
    rTd=robot.T(1:3,1:3);
    pTd=robot.T(1:3,4);    

    % set up storage space
    q0=robot.q;
    n=length(q0); % # of joints
    q=zeros(n,N+1); 
    q(:,1)=q0; % output joint displacements
    pT=zeros(3,N+1); % output p
    rT=zeros(3,3,N+1); % output R

    % iterative update
    for i=1:N
        % forward kinematics
        robot.q=q(:,i);
        robot=nlinkfwdkin(robot);
        rT(:,:,i)=robot.T(1:3,1:3);
        pT(:,i)=robot.T(1:3,4);  
        % task space error: orientation error and position error 
        % Using eR2 = 4*(1-q0) where eR2dot = (2qv)^T * omega to update
        % orientation error
        Er = rT(:,:,i)*rTd';  quat = R2q(Er);
        qv = quat(2:4);
        s = 2*qv;  
        stateError = [s; pT(:,i)-pTd];
        % Jacobian update - note 10 times the gain for orientation        
        % qq=q(:,i)-alpha*pinv(robot.J)*dX;
        qq=q(:,i)-alpha*robot.J'*inv(robot.J*robot.J'+.01*diag(1./weights))*stateError;
        q(:,i+1)=(qq>pi).*(-2*pi+qq)+(qq<-pi).*(2*pi+qq)+(qq<pi).*(qq>-pi).*qq;
    end
    % final iteration
    robot.q=q(:,N+1);
    robot=nlinkfwdkin(robot);
end

