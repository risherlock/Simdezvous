function [R] = quat_to_dcm(q)
% Quaternion to rotation matrix
%
% Convention: psi rotation about v axis.
%   q = [q0, q1, q2, q3]'
%     = [cos(0.5*psi), sin(0.5*psi) * v']'

  q = [q(4), q(1), q(2), q(3)]';

  R = zeros(3);
  R(1,1) = q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2;
  R(1,2) = 2*(q(2)*q(3) + q(1)*q(4));
  R(1,3) = 2*(q(2)*q(4) - q(1)*q(3));
  R(2,1) = 2*(q(2)*q(3) - q(1)*q(4));
  R(2,2) = q(1)*q(1) - q(2)*q(2) + q(3)*q(3) - q(4)*q(4);
  R(2,3) = 2*(q(3)*q(4) + q(1)*q(2));
  R(3,1) = 2*(q(2)*q(4) + q(1)*q(3));
  R(3,2) = 2*(q(3)*q(4) - q(1)*q(2));
  R(3,3) = q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2;
end
