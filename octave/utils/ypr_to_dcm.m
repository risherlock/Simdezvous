function R = ypr_to_dcm(ypr)
  cy = cos(ypr(1));
  cp = cos(ypr(2));
  cr = cos(ypr(3));
  sy = sin(ypr(1));
  sp = sin(ypr(2));
  sr = sin(ypr(3));

  R = zeros(3, 3);
  R(1,1) = cp * cy;
  R(1,2) = cp * sy;
  R(1,3) = -sp;
  R(2,1) = sr * sp * cy - cr * sy;
  R(2,2) = sr * sp * sy + cr * cy;
  R(2,3) = sr * cp;
  R(3,1) = cr * sp * cy + sr * sy;
  R(3,2) = cr * sp * sy - sr * cy;
  R(3,3) = cr * cp;
end
