function [q] = qprod(q1, q2)
% Hamilton's quaternion multiplication, Eqn(A.8b)
% q = q1 * q2

  Xi = [q1(4) * eye(3) + skew(q1(1:3)); -q1(1:3)']; % Eqn(A.9b)
  q = [Xi, q1] * q2; % Eqn(A.8b)
end
