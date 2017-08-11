clear all; close all; clc

z0(1) = 100;
A = [1  1 1;
     100 -1 20*z0;
    -3  5 8];
b = [3; 119; 10];
tol = 10;
iter = 1;

tic
while abs(tol) > 0.00001
    sol = A \ b;
    A(2, 3) = 20 * sol(3);
    tol(iter) = sol(3) - z0(iter);
    iter = iter + 1;
    z0(iter) = sol(3);
end
time_take = toc

sol
plot(tol); grid on
figure()
plot(z0); grid on;