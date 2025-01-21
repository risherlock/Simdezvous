clc
clear
close all

addpath('utils');

% Simulation params [s]
start_adcs = 0;
stop_adcs = 900;
start_mag = stop_adcs;
stop_mag = 500;
dt = 0.1;
time = start_adcs : dt : stop_mag;

% Magnet params
mu0 = 1.25663706e-6; % m * kg / (s^2 * A^2)
n = 570; % Number of turns
i = 3; % Max current [amp]
m = 4 * [mu0 * n * i, 0, 0]';

% Earth's magnetic field
B_unit = randn(3, 1);
B =  (B_unit / norm(B_unit));

% MOI tensor
mass = 7; % kg
x = 0.10; % m
y = 0.10; % m
z = 0.30; % m
I = (mass/12) * diag([y^2 + z^2,  z^2 + z^2, x^2 + y^2]); % kg m^2

% Control gains
kp = 0.001;
kd = 0.01;

% Initial
q0 = [0.685, 0.695, 0.153, 0.153]';
w0 = deg2rad([-0.53, 0.53, 0.053]'); % rad/s

% desired state
xVector = [1;0;0];
angle_d = acos(dot(xVector, B) / (norm(xVector) * norm(B)));
axis_d = cross(xVector,B);
sinAxis = sin(angle_d/2);
qd = [sinAxis*axis_d(1), sinAxis*axis_d(2), sinAxis*axis_d(3),cos(angle_d/2)]';
qd=qd/norm(qd)
norm(qd)

% Memory allocation
state = zeros(7, length(time));
state(:,1) = [q0; w0];
q_err = zeros(4, length(time));

% Attitude control
for t = 1:length(time)-1
  q = state(1:4, t);
  w = state(5:7, t);

  % if(time(t) < stop_adcs)
    % Control torque
    dq = qerr(qd, q);
  
    if(dq(4) < 0)
      dq = -dq;
    end
  
    q_err(:,t) = dq;
    tau = -kp * sign(dq(4)) * dq(1:3) - kd * w;
  % else
    % tau = cross(m , quat_to_dcm(q) * B);
  % endif

  % Numerical integration
  fn = @(state)dynamics(state, I, tau);
  state(:, t+1) = rk4(fn, state(:,t), dt);

  % Quaternion normalization
  state(1:4, t+1) = state(1:4, t+1) / norm(state(1:4, t+1));
end

% Quaternions
figure;
subplot(4,1,1); plot(time, state(1,:)); grid on; ylabel('q_{1}'); xlabel('Time [s]'); title("Quaternion profile");
subplot(4,1,2); plot(time, state(2,:)); grid on; ylabel('q_{2}'); xlabel('Time [s]');
subplot(4,1,3); plot(time, state(3,:)); grid on; ylabel('q_{3}'); xlabel('Time [s]');
subplot(4,1,4); plot(time, state(4,:)); grid on; ylabel('q_{0}'); xlabel('Time [s]');

% Error Quaternions
figure;
subplot(4,1,1); plot(time, q_err(1,:)); grid on; ylabel('q_err{1}'); xlabel('Time [s]'); title("Quaternion Error profile");
hold on; plot(time, zeros(size(time)));
subplot(4,1,2); plot(time, q_err(2,:)); grid on; ylabel('q_err{2}'); xlabel('Time [s]');
hold on; plot(time, zeros(size(time)));
subplot(4,1,3); plot(time, q_err(3,:)); grid on; ylabel('q_err{3}'); xlabel('Time [s]');
hold on; plot(time, zeros(size(time)));
subplot(4,1,4); plot(time, q_err(4,:)); grid on; ylabel('q_err{0}'); xlabel('Time [s]');
hold on; plot(time, ones(size(time)));

% Angular rates
figure;
plot(time, state(5,:)); hold on;
plot(time, state(6,:)); grid on;
plot(time, state(7,:));
title('Angular rate (rad/s)');
legend('\omega_1', '\omega_2', '\omega_3');
xlabel('Time [s]'); ylabel('rad/s');
