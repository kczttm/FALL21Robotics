clear all;close all;

% define a pinhole camera

cam_def;

% define target points in the inertial frame

target_def;

% Specify # of images
M=10;

% calibration matrix
L=zeros(2*M,6);

% build up the calibration matrix
for i= 1:M
    % Randomly generate R_{oc} (p_{0c} set so targets are in front of camera)
    theta=rand*pi/10;k=randn(3,1);k=k/norm(k);
    p=[.8*(rand-.5);.8*(rand-.5);-(rand+1)*3];
    Toc{i} = [rot(k,theta) p; 0 0 0 1];
    
    % projection onto camera image plane
    %[uv{i},uvw{i},P1]=cam_image(cam,Toc{i},P0);
    [uv{i},uvw{i},P1]=cam_image_round(cam,Toc{i},P0);
    
    if size(uv{i},2)<N;
        disp('some patterns are out of view');
        disp(uv{i});i=i-1;
        return;
    end
    
    % ns pixel error
    ns=.5;
    uv{i}=uv{i}+randn(size(uv{i}))*ns;
    
    % find the Homography matrix
    
    Hmat{i}=homographymat(uv{i},Pxy);
    hx=Hmat{i}(:,1);
    hy=Hmat{i}(:,2);
    L(2*i-1,:)=[hx(1)^2-hy(1)^2 hx(2)^2-hy(2)^2 hx(3)^2-hy(3)^2 ...
        2*(hx(1)*hx(2)-hy(1)*hy(2)) 2*(hx(1)*hx(3)-hy(1)*hy(3)) ...
        2*(hx(2)*hx(3)-hy(2)*hy(3))];
    L(2*i,:)=[hx(1)*hy(1) hx(2)*hy(2) hx(3)*hy(3) ...
        hx(1)*hy(2)+hx(2)*hy(1) hx(1)*hy(3)+hx(3)*hy(1) ...
        hx(2)*hy(3)+hx(3)*hy(2)];    
end

[U,S,V]=svd(L);
Bvec=V(:,6);
Best=[Bvec(1) Bvec(4) Bvec(5);Bvec(4) Bvec(2) Bvec(6);Bvec(5) Bvec(6) Bvec(3)];
KKtr=inv(Best);
KKtr=KKtr/KKtr(3,3);
%
tt=eye(3,3);
tt=tt(3:-1:1,:);
X=chol(tt*KKtr*tt);
Kest=tt*X'*tt;
camest=cam;
camest.K=Kest;
disp('K vs. K_est');
disp(cam.K);
disp(Kest);
for i=1:M
  [uv1{i},uvw1{i},P1]=cam_image(camest,Toc{i},P0);
  figure(M);
  plot(uv{i}(1,:),uv{i}(2,:),'x',uv1{i}(1,:),uv1{i}(2,:),'o','linewidth',3);
  title('reprojection based on camera calibration');
  grid on;
  view(180,-90);
  axis([0 W 0 H]);
  erruv{i}=uv{i}-uv1{i};
  erruvnorm(i)=norm(erruv{i});
  erruvmax(i)=max(max(abs(erruv{i})));
end
hold off