clc
clear 
close all

addpath('utils')


function v_new = quaternion_rotate_vector(q, v)
    % Rotate a vector v by a quaternion q
    q_w = q(1);
    q_x = q(2);
    q_y = q(3);
    q_z = q(4);
    
    v_x = v(1);
    v_y = v(2);
    v_z = v(3);
    
    % Quaternion multiplication: q * v * q_conj
    v_new = [
        (1 - 2*q_y^2 - 2*q_z^2) * v_x + (2*q_x*q_y - 2*q_z*q_w) * v_y + (2*q_x*q_z + 2*q_y*q_w) * v_z;
        (2*q_x*q_y + 2*q_z*q_w) * v_x + (1 - 2*q_x^2 - 2*q_z^2) * v_y + (2*q_y*q_z - 2*q_x*q_w) * v_z;
        (2*q_x*q_z - 2*q_y*q_w) * v_x + (2*q_y*q_z + 2*q_x*q_w) * v_y + (1 - 2*q_x^2 - 2*q_y^2) * v_z
    ];
end

% Earth's magnetic field
B_unit = [13140.840820,-183.549347,29122.017578];
B =  (B_unit / norm(B_unit))


% desired state
qcurrent = [0;1;0;0]
xVector = [1;0;0]';
angle_d = acos(dot(xVector, B) / (norm(xVector) * norm(B)));
axis_d = cross(xVector,B);
sinAxis = sin(angle_d/2);
qd = [sinAxis*axis_d(1), sinAxis*axis_d(2), sinAxis*axis_d(3),cos(angle_d/2)]'
qd = qprod(qd, qcurrent)
qd=qd/norm(qd);

% check if correct
alignment = dot(quaternion_rotate_vector(qd,xVector), B)
% alignment = dot(B,qprod(qd,qprod(qcurrent,qinv(qd)))) 

% quatmultiply(qd,[1,0,0,0])

