%
% PUMA 560 forward kinematics calculated using 3 different conventions
%
close all; clear all;

syms q1 q2 q3 q4 q5 q6 real
syms l1 l2 b1 b2 b3 b4 real
ex=[1;0;0];ey=[0;1;0];ez=[0;0;1];zz=[0;0;0];

% PE
puma560.H=[ez -ey -ey ez -ey ez];
puma560.P=[b1*ez zz l1*ex-b2*ey -l2*ex+b3*ez zz zz b4*ez];
puma560.q=[q1;q2;q3;q4;q5;q6];
puma560.joint_type=zeros(1,6);
puma560=fwdkiniter(puma560);

T_0T_POE = puma560.T;

% SDH
puma560.d=[b1,0,b2,b3,0,b4];
puma560.a=[0,l1,-l2,0,0,0];
puma560.alpha=sym([pi/2,0,-pi/2,pi/2,-pi/2,0]);
puma560.theta=[q1,q2,q3,q4,q5,q6];
puma560=fwdkinsdh(puma560);

T_06_SDH = puma560.T;

% MDH
puma560.d=[b1,0,b2,b3,0,0,b4];
puma560.a=[0,0,l1,-l2,0,0,0];% a_{i-1}
puma560.alpha=sym([0,pi/2,0,-pi/2,pi/2,-pi/2,0]);
puma560.theta=[q1,q2,q3,q4,q5,q6,0];;
puma560=fwdkinmdh(puma560);

T_07_MDH = puma560.T;

%
% check
%

disp(' *** T_POE - T_SDH *** ');
disp(simplify(T_0T_POE-T_06_SDH));

disp(' *** T_POE - T_MDH *** ');
disp(simplify(T_0T_POE-T_07_MDH));