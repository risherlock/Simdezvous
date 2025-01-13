
% Solution of Example 7.1
% Markley, Crassidis - Fundamentals of spacecraft AD&C (2014)

clc
clear
close all

addpath('utils');

% Simulation params [s]
start_time = 0;
stop_time = 300;
dt = 1;
time = start_time : dt : stop_time;

% Physical Parameters [kg m^2]
I = [10000, 0, 0; 0, 9000, 0; 0, 0, 12000];

% Control gains
kp = 50;
kd = 500;

% Initial and desired state
q0 = [0.685, 0.695, 0.153, 0.153]';
w0 = deg2rad([-0.53, 0.53, 0.053]'); % rad/s
qd = [0, 0, 0, 1]';

% Memory allocation
state = zeros(7, length(time));
state(:,1) = [q0; w0];

% Integration loop
for t = 1:5
  q = state(1:4, t);
  w = state(5:7, t)

  % Control torque
  dq = qerr(qd, q);
  tau = -kp * sign(dq(4)) * dq(1:3) - kd * w

  % Numerical integration
  fn = @(state)dynamics(state, I, tau);
  state(:, t+1) = rk4(fn, state(:,t), dt);


  % Quaternion normalization
  state(1:4,t+1)
  state(1:4, t+1) = state(1:4, t+1) / norm(state(1:4, t+1));
  state(1:4,t+1)
end

% Quaternions
figure;
subplot(4,1,1); plot(time, state(1,:)); grid on; ylabel('q_{1}'); xlabel('Time [s]'); title("Quaternion profile");
subplot(4,1,2); plot(time, state(2,:)); grid on; ylabel('q_{2}'); xlabel('Time [s]');
subplot(4,1,3); plot(time, state(3,:)); grid on; ylabel('q_{3}'); xlabel('Time [s]');
subplot(4,1,4); plot(time, state(4,:)); grid on; ylabel('q_{0}'); xlabel('Time [s]');

% Angular rates
figure;
plot(time, state(5,:)); hold on;
plot(time, state(6,:)); grid on;
plot(time, state(7,:));
title('Angular rate (rad/s)');
legend('\omega_1', '\omega_2', '\omega_3');
xlabel('Time [s]'); ylabel('rad/s');
