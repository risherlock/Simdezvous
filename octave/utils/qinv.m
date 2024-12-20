% Inverse of a quaternion
function [q_inv] = qinv(q)
  q_inv = qconj(q) / norm(q)^2;
end