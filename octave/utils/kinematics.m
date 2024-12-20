function [q_dot] = kinematics(q, w)
% Quaternion kinematics differential equation

  q_dot = 0.5 * qprod(q, [w; 0]);
end
