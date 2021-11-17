%
% miniproject 5 part 4
%
% generate estimate of T_0B and T_TC
%
hand_eye_cal; % from part 3

% set up PBVS call

irb1200.q=irb1200.q.*(1+randn(6,1)*.05);

N=30;
irb1200.MaxIter=N; % # of iteration for kinematic control
irb1200.StepSize=.3;   % controller gain
irb1200.Weights=[1;1;1;10;10;10]; % weighting factors

% target camera frame with respect to reference frame 0
T_0C_des=[-ey ez -ex [1.8;0;0.55];0 0 0 1];
% PBVS to reduce error
[q,uv,T_0C]=PBVS_unlimited_range(irb1200,cam,pS,T_0C_des,T_0B,T_TC,T_0B_est,T_TC_est);

disp(T_0C{N+1})
disp(T_0C_des)

% show image space progression
for i=1:N 
    H=figure(50);plot(uv{i}(1,:),uv{i}(2,:),'x','linewidth',2);
    xlabel('image plane horizontal axis');
    ylabel('image plane vertical axis'); 
    axis([0 cam.uv0(1)*2 0 cam.uv0(2)*2]);
    set ( gca, 'xdir', 'reverse' );%pause
    %drawnow;
    drawnow;M_Image(i)=getframe(H);
end

H=figure(60);movie(H,M_Image,1,4);
