clc
clear
close all

data = csvread('output.csv');

t = data(:, 1);
q = data(:, 2:5);
w = data(:, 6:8);
qe = data(:, 9:12);
tau = data(:, 13:15);

figure;
subplot(4,1,1); plot(t, q(:,1), 'lineWidth', 1); axis tight; title({'Quaternion profile', 'q1'}); grid on; xlabel('time [s]');
subplot(4,1,2); plot(t, q(:,2), 'lineWidth', 1); axis tight; title('q2'); grid on; xlabel('time [s]');
subplot(4,1,3); plot(t, q(:,3), 'lineWidth', 1); axis tight; title('q3'); grid on; xlabel('time [s]');
subplot(4,1,4); plot(t, q(:,4), 'lineWidth', 1); axis tight; title('q0'); grid on; xlabel('time [s]');

figure;
subplot(4,1,1); plot(t, qe(:,1), 'lineWidth', 1); axis tight; title({'Error quaternion profile', '\delta q1'}); grid on; xlabel('time [s]');
subplot(4,1,2); plot(t, qe(:,2), 'lineWidth', 1); axis tight; title('\delta q2'); grid on; xlabel('time [s]');
subplot(4,1,3); plot(t, qe(:,3), 'lineWidth', 1); axis tight; title('\delta q3'); grid on; xlabel('time [s]');
subplot(4,1,4); plot(t, qe(:,4), 'lineWidth', 1); axis tight; title('\delta q0'); grid on; xlabel('time [s]');

figure;
subplot(3,1,1); plot(t, w(:,1), 'lineWidth', 1); axis tight; title({'Angular rate', 'w1'}); grid on; xlabel('time [s]'); ylabel('[rad/s]');
subplot(3,1,2); plot(t, w(:,2), 'lineWidth', 1); axis tight; title('w2'); grid on; xlabel('t[s]'); ylabel('[rad/s]');
subplot(3,1,3); plot(t, w(:,3), 'lineWidth', 1); axis tight; title('w3'); grid on; xlabel('t[s]'); ylabel('[rad/s]');

figure;
subplot(3,1,1); plot(t, tau(:,1), 'lineWidth', 1); axis tight; title({'Command torque', '\tau 1'}); grid on; xlabel('time [s]'); ylabel('[rad/s]');
subplot(3,1,2); plot(t, tau(:,2), 'lineWidth', 1); axis tight; title('\tau 2'); grid on; xlabel('t[s]'); ylabel('[rad/s]');
subplot(3,1,3); plot(t, tau(:,3), 'lineWidth', 1); axis tight; title('\tau 3'); grid on; xlabel('t[s]'); ylabel('[rad/s]');

pause;
