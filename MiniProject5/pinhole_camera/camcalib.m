function camcalib(M,npoints)
% define a pinhole camera

cam_def;

% define target points in the inertial frame

[P0, Pxy, N] = target_def(npoints);

% Specify # of images
%M=20;

% calibration fit matrix
L=zeros(2*M,6);

% build up the calibration fit matrix
for i= 1:M
    
    % set image generation flag to 1
    image_gen_flag = 1;
    
    while image_gen_flag > 0
        
        % Randomly generate R_{oc} (p_{0c} set so targets are in front of camera)
        theta=rand*pi/10;k=randn(3,1);k=k/norm(k);
        p=[.8*(rand-.5);.8*(rand-.5);-(rand+1)*3];
        Toc{i} = [rot(k,theta) p; 0 0 0 1];
        
        % projection onto camera image plane
        [uv_true{i},uvw_true{i},Pc_true{i}]=cam_image(cam,Toc{i},P0);
        %[uv_true{i},uvw_true{i},P1]=cam_image_round(cam,Toc{i},P0);
        
        if size(uv_true{i},2)<N;
            disp('some patterns are out of view');
        else
            image_gen_flag = 0;
        end
    
    end
    
    % ns pixel error
    ns=.1;
    uv_meas{i}=uv_true{i}+randn(size(uv_true{i}))*ns;
    
    % find the Homography matrix
    
    Hmat{i}=homographymat(uv_meas{i},Pxy);
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
svalues=sprintf(',  %0.4g',diag(S));
disp(['**** Singular Values ****: [',svalues(2:end),'  ]'])
Bvec=V(:,6);
Best=[Bvec(1) Bvec(4) Bvec(5);Bvec(4) Bvec(2) Bvec(6);Bvec(5) Bvec(6) Bvec(3)];
KKtr=inv(Best);
KKtr=KKtr/KKtr(3,3);
%
tt=eye(3,3);
tt=tt(3:-1:1,:);
X=chol(tt*KKtr*tt);
% Estimated intrinsic camera matrix
K_est=tt*X'*tt;
% Define the identified camera
cam_est=cam;
cam_est.K=K_est;
% compare the intrinsic camera matrix
disp('K');
disp(cam.K);
disp('Estimated K');
disp(cam_est.K);
disp('percentage error in K');
cc=sprintf(' %0.2g, %0.2g, %0.2g\n',(abs((cam.K-cam_est.K)./cam.K)*100)');
cc=strrep(cc,'NaN','-');
cc=strrep(cc,'Inf','-');
disp(cc);
%  reproject plot
% 
% plot color array
plot_color=jet(M);zmax=0;
% 
for i=1:M    
  % estimate T_co
  
  Ti=inv(cam_est.K)*Hmat{i};
  ai=mean([norm(Ti(:,1)) norm(Ti(:,2))]);
  Rpi=Ti/ai;
  Rpi=sign(Rpi(3,3))*Rpi;
  Tco_est=[Rpi(:,1) Rpi(:,2) cross(Rpi(:,1),Rpi(:,2)) Rpi(:,3);0 0 0 1];
  Toc_est{i}=inv(Tco_est);
  [uv_est{i},uvw_est{i},Pc_est{i}]=cam_image(cam_est,Toc_est{i},P0);
 
  % error 
  erruv{i}=uv_true{i}(:,1:size(uv_est{i},2))-uv_est{i};
  erruvnorm(i)=norm(erruv{i});
  erruvmax(i)=max(max(abs(erruv{i})));
  errPc{i}=Pc_true{i}-Pc_est{i};
  errPcnorm(i)=norm(errPc{i});
  errPcmax(i)=max(max(abs(errPc{i})));
end

%  plot in 3D 
figure(1000);
for i=1:M
  plot3(Pc_true{i}(1,:),Pc_true{i}(2,:),Pc_true{i}(3,:),'x','linewidth',1,...
      'color',plot_color(i,:));
  hold on;
  plot3(Pc_est{i}(1,:),Pc_est{i}(2,:),Pc_est{i}(3,:),'o','linewidth',1,...
      'color',plot_color(i,:));
end
xlabel('x-axis');ylabel('y-axis');zlabel('z-axis');
title('3D reprojection based on camera calibration');
grid on;
zz=zlim(gca);
zmax=max(zz(2),zmax);
view([-96,-43])
axis([xlim(gca) ylim(gca) 0 zmax])
hold off

% plot in image plane
figure(2000);
for i=1:M
  plot(uv_true{i}(1,:),uv_true{i}(2,:),'x',...
      uv_est{i}(1,:),uv_est{i}(2,:),'o','linewidth',3);
  hold on
end
title('image plane reprojection based on camera calibration');
grid on;
view(180,-90);
axis([0 W 0 H]);
hold off
% max reprojection error
mxerr=sprintf(',  %4.3g',max(erruvmax));
%mxerr=sprintf(',  %4.3g',max(errPcmax));
disp(['**** Maximum Reprojection Error:  ',mxerr(2:end)])
hold off