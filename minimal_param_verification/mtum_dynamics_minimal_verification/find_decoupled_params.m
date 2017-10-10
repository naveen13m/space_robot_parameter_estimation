% Computes decoupled parameter set which is equivalent to the given
% parameter set 
clear all; close all; clc;

global actual_params
actual_params = [1200, 1000, 0, 0, 30, 75, 0.5, 0, 10, 50, 0.5, 0];
%     Izz0c   m0  0a0x 0a0y Izz1c   m1  1a1x  1a1y   Izz2c   m2  2a2x  2a2y
lb = [   0,    0,   0,   0,    0,    0,    0,   0,      0,    0,    0,   0];
ub = [5000, 5000,   0,   0,  500,  200,  0.5,   0,    500,  200,  0.5,   0];
x0 = [1000,  550,   0,   0,    10,   10,  0.5,   0,      10,   10,  0.5,   0];
[alt_params, error] = fmincon(@error_coupled_params, x0, [], [], [], [], lb, ub)

save sols.mat actual_params alt_params