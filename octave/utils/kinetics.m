function [w_dot] = kinetics(w, I, tau)
% Euler's equation of rotational dynamics

  w_dot = I \ (-cross(w, I * w) + tau);
end
