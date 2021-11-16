%
% homographymat.m
%
% Find the homography matrix 
%
% usage: H=homographymat(uv,Pxy)
%
% input:
%   uv: image pixel coordinate of N points (2xN matrix)
%   Pxy: planar set of points in the inertial frame
% 
% output:
%   H: Homography matrix
%

function H=homographymat(uv,Pxy)

N=size(Pxy,2);
A=zeros(2*N,9);
z3=zeros(1,3);
alpha=[uv;ones(1,N)];
for i=1:N
    beta=[Pxy(:,i);1];
    A(2*(i-1)+1:2*i,:)=...
        [z3 -alpha(3,i)*beta' alpha(2,i)*beta';...
        alpha(3,i)*beta' z3 -alpha(1,i)*beta'];
end
[U,S,V]=svd(A);h=V(:,9);
%h=-null(A);
H=[h(1:3)';h(4:6)';h(7:9)'];
H=H/norm(H(:,1));

end
