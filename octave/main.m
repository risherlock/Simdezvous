clc
clear
close all

% MOI tensor
m = 1.5; % kg
x = 0.10; % cm
y = 0.10; % cm
z = 0.30; % cm
I = (m/12) * diag([y^2 + z^2,  z^2 + z^2, x^2 + y^2]); % kg m^2
