%
% kincheckelbow.m
%
% forward/inverse fkinematics checker for an elbow arm
% using MATLAB inversekinematics solver vs. subproblem decomposition
% 
% make sure fwdkin_example_rbt is in the path to generate the elbow_rbt
% rigid body tree
% 
% output plots compare EE solution accuracy and computation times
%

%clear all; close all;
clearvars
fwdkin_examples_rbt;
%elbow.P(:,1)=[.1;.2;.3];
% MATLAB robotics toolbox inverse kinematics object
ik = inverseKinematics('RigidBodyTree',elbow_rbt);

% set storage
N=200;

n=length(elbow.H);
q=zeros(n,N);
qsol=zeros(n,N);
qsol1=zeros(n,8,N);
T=zeros(4,4,N);
Tsol=zeros(4,4,N);
Tsol1=zeros(4,4,8,N);
Jsol1=zeros(6,6,8,N);
errT=zeros(N,1);
errT1=zeros(N,8);
telapsed=zeros(1,N);
telapsed1=zeros(1,N);

for i=1:N
    % random arm configuration
    q(:,i)=(rand(6,1)-.5)*pi;
    elbow.q=q(:,i);
    % forward kinematics
    elbow=fwdkiniter(elbow);
    % nominal end effector pose
    T(:,:,i)=elbow.T;
    % MATLAB's inverse kinematics
    % keep track of the computation time
    tstart=tic;
    [qsol(:,i),solnInfo]=...
        ik('body7',elbow.T,ones(1,6),q(:,i)+q(:,i)*.1*randn);
    telapsed(i) = toc(tstart);
    % forward kinematics again to compare with EE pose
    elbow.q=qsol(:,i);
    elbow=fwdkiniter(elbow);
    Tsol(:,:,i)=elbow.T;
    % now use our own exact inverse kinematics
    elbow.q=q(:,i);
    elbow.T=T(:,:,i);
    % keep track of the computation time
    tstart=tic;
    solns=invkinABB(elbow);
    telapsed1(i) = toc(tstart);
    % there are 8 solutions
    qsol1(:,:,i)=solns';
    % find all the EE pose
    for j=1:8;
        elbow.q=qsol1(:,j,i);
        elbow=fwdkiniter(elbow);
        Tsol1(:,:,j,i)=elbow.T;
        elbow=fwddiffkiniter(elbow);
        Jsol1(:,:,j,i)=elbow.J;
        errT1(i,j)=norm(T(:,:,i)-Tsol1(:,:,j,i),'fro');
    end
    % find EE pose error
    errT(i)=norm(T(:,:,i)-Tsol(:,:,i),'fro');   
end 

figure(10);plot(errT,'bx','linewidth',2); hold on;
for j=1:8
    plot(errT1(:,j),'ro','linewidth',2);
end
hold off;
xlabel('random test number');ylabel('end effector error');
title('End effector pose error || T - T_{solve} ||_F');
legend('MATLAB','exact by subproblem');

fprintf('max MATLAB EE error: %g \n',max(errT));
fprintf('max subproblem EE error: %g \n',max(max(errT1)));

figure(20);plot((1:N),telapsed,'bx',(1:N),telapsed1,'ro','linewidth',2);
xlabel('random test number');ylabel('elapsed time (sec)');
title('inverse kinematics computation time');
legend('MATLAB','exact by subproblem');

fprintf('max and average MATLAB invkin computation time: %g, %g \n',...
    max(telapsed),mean(telapsed));
fprintf('max and average subproblem invkin computation time: %g, %g \n',...
    max(telapsed1),mean(telapsed1));

