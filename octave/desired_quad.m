clc
clear 
close all

addpath('utils')

% Earth's magnetic field
B_unit = [13140.840820,-183.549347,29122.017578];
B =  (B_unit / norm(B_unit))


% desired state
xVector = [1;0;0]';
angle_d = acos(dot(xVector, B) / (norm(xVector) * norm(B)));
axis_d = cross(xVector,B);
sinAxis = sin(angle_d/2);
qd = [sinAxis*axis_d(1), sinAxis*axis_d(2), sinAxis*axis_d(3),cos(angle_d/2)];
qd=qd/norm(qd)
norm(qd)

% check if correct
R = quat_to_dcm(qd)
Y = R*B'

Z=R'*xVector'

% quatmultiply(qd,[1,0,0,0])
