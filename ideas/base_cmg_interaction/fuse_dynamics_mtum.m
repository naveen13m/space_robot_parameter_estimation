clear all; close all; clc;

curr_mat = load('global_kin_mat_case6.mat');
curr_mat = remove_excess(curr_mat.global_kin_mat_1);
K0 = curr_mat(:, 1 : 4);% + 0.001 * rand(300, 4);
Kr = curr_mat(:, 5 : 8);% + 0.001 * rand(300, 4);

dth0 = zeros(100, 1);
al0 = zeros(100, 1);
al1 = zeros(100, 1);
acc_x0 = zeros(100, 1);
acc_y0 = zeros(100, 1);
dth0(1) = K0(3, 1);
al0(1) = (K0(3, 1) - 0) / 0.01;
al1(1) = (Kr(3, 1) - 0) / 0.01;
acc_y0(1) = (K0(1, 2) - 0) / 0.01;
acc_y0(1) = (K0(2, 2) - 0) / 0.01;
reg_mat_dyn = zeros(300, 4);
ax0(1) = 10; ay0(1) = 10;
reg_mat_dyn(1 : 3, :) = [0      acc_x0(1) 0                 0;
                         0      acc_y0(1) 0                 0;
                         al0(1) 0         -ax0(1) * al0(1)  -ay0(1) * al0(1)];
for i = 2 : 100
    dth0(i) = K0(3 * i, 1);
    al0(i) = (K0(3 * i, 1) - K0(3 * i - 3, 1)) / 0.01;
    al1(i) = (Kr(3 * i, 1) - Kr(3 * i - 3, 1)) / 0.01;
    acc_x0(i) = (K0(3 * i - 2, 2) - K0(3 * i - 5, 2)) / 0.01;
    acc_y0(i) = (K0(3 * i - 1, 2) - K0(3 * i - 4, 2)) / 0.01;
    reg_mat_dyn(3 * i - 2 : 3 * i, :) = [0      acc_x0(i) 0                0;
                                         0      acc_y0(i) 0                0;
                                         al0(i) 0         -ax0(1) * al0(i) -ay0(1) * al0(i)];
end

tol = 100;
rhs_mtum = -Kr * [10; 20; 0; 0];
rhs_dyn(1 : 3 : 300, 1) =  -20 * (acc_x0 - 0.5 * dth0.^2);
rhs_dyn(2 : 3 : 300, 1)  =  -20 * (acc_y0 + 0.5 * al0);
rhs_dyn(3 : 3 : 300, 1)  = -(10 * al1 + 20 * 0.25 * al0 + 20 * 0.5 * acc_y0);

basic_sol = pinv(reg_mat_dyn(:, 1 : 2)) *  rhs_dyn;

rhs = [rhs_mtum; rhs_dyn];
reg_mat = [K0; reg_mat_dyn];
iter = 1;

while abs(tol(iter)) > 0.000001 && iter < 100
    iter
    cn(iter) = cond(reg_mat);
    iter = iter + 1;
    sol(:, iter) = pinv(reg_mat) * rhs;
    ax0(iter) = sol(3, iter) / sol(2, iter);
    ay0(iter) = sol(4, iter) / sol(2, iter);
    reg_mat_dyn(3 : 3 : end, 3) = reg_mat_dyn(3 : 3 : end, 3) * (ax0(iter) / ax0(iter - 1));
    reg_mat_dyn(3 : 3 : end, 4) = reg_mat_dyn(3 : 3 : end, 4) * (ay0(iter) / ay0(iter - 1));
    reg_mat(301 : 600, :) = reg_mat_dyn;
    tol(iter) = norm([ax0(iter) - ax0(iter - 1), ay0(iter) - ay0(iter - 1)]);
end
 
sol
check = rref(reg_mat, 10^-3);
plot(ax0); hold on;
plot(ay0); legend('x-com', 'y-com')
figure();
plot(cn);