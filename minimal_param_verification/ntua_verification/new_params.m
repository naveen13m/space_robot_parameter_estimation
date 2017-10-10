clear all; close all; clc;

%     m0     m1      m2     r0x r0y  r1  r2    I0      I1     I2
x0 = [200,     550,    350, 0.5, 0, 0.1, 0.1,  500,    200,   200];
lb = [0,         0,      0, 0.5, 0,   0,   0,    0,      0,     0];
ub = [10000, 10000,  10000, 0.5, 0,   1,   1, 10000, 10000, 10000];
error_fun = @ntua_minimal_params;
[params, error] = fmincon(error_fun, x0, [], [], [], [], lb, ub);

