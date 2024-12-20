function [ypr] = quat_to_ypr(q)
% Quaternion to Euler angles in 3-2-1 sequence [rad]

  q0 = q(1);
  q1 = q(2);
  q2 = q(3);
  q3 = q(4);

  ypr = zeros(3,1);
  ypr(1) = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
  ypr(2) = asin(2 * (q0 * q2 - q3 * q1));
  ypr(3) = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
end
