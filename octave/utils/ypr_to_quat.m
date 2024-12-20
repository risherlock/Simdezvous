function [q] = ypr_to_quat(ypr)
% Euler (3-2-1) [rad] to quaternion

  cy = cos(0.5 * ypr(1));
  sy = sin(0.5 * ypr(1));
  cp = cos(0.5 * ypr(2));
  sp = sin(0.5 * ypr(2));
  cr = cos(0.5 * ypr(3));
  sr = sin(0.5 * ypr(3));

  % Compute quaternion components
  q = zeros(4, 1);
  q(1) = cy * cp * cr + sy * sp * sr;
  q(2) = cy * cp * sr - sy * sp * cr;
  q(3) = sy * cp * sr + cy * sp * cr;
  q(4) = sy * cp * cr - cy * sp * sr;
end
