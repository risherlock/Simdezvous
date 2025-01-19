clc
clear
close all

% for H = 5 Oe
function output = gauss_oe_to_tesla_m_per_amp (input)
    gauss_to_tesla = 10^(-4)
    oe_to_amp_per_meter = 5 * 1000/(4*pi) % for H = 5 Oe
    output = input*gauss_to_tesla/(oe_to_amp_per_meter) 
end 

N = 570; % no of turns
L = (60.1-12.1-3)*10^(-3); %length of solenoid in m
mu = gauss_oe_to_tesla_m_per_amp(1600); %permeability for 10 degrees
n = N/L;
I = 3; %A

B=mu*n*I % Tesla

