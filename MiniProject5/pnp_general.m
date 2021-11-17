%
% pnp_general.m: solution of the general PnP problem
%
% **** Note: this does not work for planar targets (null space is dimension
% 4, and an additional fit is necessary) ****  
% 
% input: 
%       uv = image coordinates in pixels of target points
%       P0 = locations of target points in reference frame
%       K = camera intrinsic calibration matrix
%
% output: 
%       Test = T_C0 estimate
%       Zest = depth estimate of each target point
% 

function [Test,Zest]=pnp_general(uv,P0,K)

% # of target points
N=size(P0,2);

% multiply image by K inverse
y=inv(K)*[uv;ones(1,N)];
% 
%y=reshape([uv;ones(1,N)],3*N,1);
%Pstack=reshape(P0,3*N,1);

% form the regression matrix
C=zeros(3*N,N+3+9);
for i=1:N
   C((i-1)*3+1:3*i,1:3*3)=kron(eye(3,3),P0(:,i)');
   C((i-1)*3+1:3*i,3*3+1:3*(3+1))=eye(3,3);
   C((i-1)*3+1:3*i,3*(3+1)+i)=y(:,i);
end

% use svd to estimate the near null space 
% (check the last few smallest singular values!)
[U,S,V]=svd(C);
% estimate null space
% null space of C is 4 dimensional if target points are in a plane
nC=V(:,12+N);
% extract orientation
R1=reshape(nC(1:9),3,3)';
% use svd to extract orthonormal components
[u,s,v]=svd(R1);
% estimate orientation matrix R_{co}
Rest=u*v'*sign(det(R1));
% find the scaling constant 
a=pinv(reshape(Rest',9,1))*nC(1:9);
% recover p_{co} and depth by scaling appropriately
pest=nC(10:12)/a;
Zest=nC(13:13+N-1)/a;
% This is the estimate T_{co}
Test=[Rest pest;0 0 0 1];
end

