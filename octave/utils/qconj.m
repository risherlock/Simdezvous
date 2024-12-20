function [q_star] = qconj(q)
% Conjugate of a quaternion

  q_star = [-q(1:3)', q(4)]';
end
