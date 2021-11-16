function [P0, Pxy, N] = target_def(numpoints)
% planar target points in the reference frame (total Nx*Ny points)
spacing = 0.4/(numpoints-1);
z0=0; % z-coordinate of the points (planar, so 0)
nx1=-.2;kx=spacing;nx2=.2; % arrays in x
ny1=-.2;ky=spacing;ny2=.2; % arrays in y
%nx1=-.2;kx=.2;nx2=.2; % arrays in x
%ny1=-.2;ky=.2;ny2=.2; % arrays in y
px=(nx1:kx:nx2);Nx=length(px);
py=(ny1:ky:ny2);Ny=length(py);
Pxy=kron([zeros(1,Nx);ones(1,Nx)],py)+...
  kron(px,[ones(1,Ny);zeros(1,Ny)]);
N = Nx*Ny;
P0=[Pxy;z0*ones(1,Nx*Ny)]; % pack into a 2x(Nx*Ny) vector
end
