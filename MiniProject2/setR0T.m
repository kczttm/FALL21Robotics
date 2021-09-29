function [xT,yT]=setR0T(Sls)
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];
% xT and yT of the T link
path_prime = diff(Sls')'./vecnorm(diff(Sls')');
yT = [path_prime; zeros(1,size(path_prime,2))];
xT = -crossmat(ez)*yT;
end
