function [state_dot] = dynamics(state, I, tau)
% 3DOF dynamical equation

  q = state(1:4);
  w = state(5:7);

  q_dot = kinematics(q, w);
  w_dot = kinetics(w, I, tau);

  state_dot = [q_dot; w_dot];
end
