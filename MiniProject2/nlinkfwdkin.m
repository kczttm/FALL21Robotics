function robotsym=nlinkfwdkin(robotsym)
% Forward kinematics works on 3D arms
% uses crossmat.m
%% Variables needed
q = robotsym.q; %  1xn joint angles
h = robotsym.H; %  3xn joint rotation axis 
l = robotsym.P(:,2:end); %  3xn joint length in its own frame
n = length(q); %  number of joint n
% place holders
pCum = zeros(3,1); 
rCum = eye(3,3);

%jCum = zeros(3,n); %  jacobian: just position pdot part here
pCum2 = zeros(3,1);
rCum2 = eye(3,3);
%% Looping to get to p0T
pCurr = robotsym.P(:,1);
pCum = pCum + pCurr;
for i =1:n
    rCurr = rot(h(:,i),q(i));
    rCum = rCum*rCurr; 
    pCurr = rCum*l(:,i);
    pCum = pCum + pCurr;
end
%% Looping for Jacobian
pCurr = robotsym.P(:,1);
pCum2 = pCum2 + pCurr;
for i = 1:n
    cross = crossmat(h(:,i));
    jCum(:,i) = cross*(pCum-pCum2); %  had to do this otherwise syms won't work
    % note that R0i*piT= p0T - p0i
    rCurr = rot(h(:,i),q(i));
    rCum2 = rCum2*rCurr; 
    pCurr = rCum2*l(:,i);
    pCum2 = pCum2 + pCurr;
end
%% Assign back to the robot object
robotsym.T(1:3,1:4) = [rCum,pCum]; %  Transformation Matrix
robotsym.J = [ones(1,n);jCum]; %  Jacobian Matrix 4xn first row all ones
end