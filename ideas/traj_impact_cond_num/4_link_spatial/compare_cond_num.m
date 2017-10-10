clear all; close all; clc;

filename = 'reg_mat_case';
num_cases = 6;

conc_reg_mat = [];
for curr_case = 1 : num_cases
    if curr_case ~= [10]
%     if curr_case ~= [4, 5, 6]
        reg_mat{curr_case} = load(strcat(filename, int2str(curr_case), '.mat'));
        cond_reg_mat_case(curr_case) = cond(reg_mat{curr_case}.reg_mat_case);
        curr_reg_mat = reg_mat{curr_case}.reg_mat_case;
        check = rref(curr_reg_mat, 10e-3);
        conc_reg_mat = [conc_reg_mat; curr_reg_mat];
        coeffs(:, curr_case) = check(1 : 26, 27);
        cond_conc_reg_mat(curr_case) = cond(conc_reg_mat);
        cond_lin_conc_reg_mat(curr_case) = cond(conc_reg_mat([1 : 6 : end, 2 : 6 : end, 3 : 6 : end], [7, 8, 9, 15, 16, 22, 23, 29, 30]));
        cond_ang_conc_reg_mat(curr_case) = cond(conc_reg_mat([4 : 6 : end, 5 : 6 : end, 6 : 6 : end], [1 : 6, 10 : 14, 17 : 21, 24 : 28]));
    else
        cond_conc_reg_mat(curr_case) = cond_conc_reg_mat(curr_case - 1);
        cond_lin_conc_reg_mat(curr_case) = cond_lin_conc_reg_mat(curr_case - 1);
        cond_ang_conc_reg_mat(curr_case) = cond_ang_conc_reg_mat(curr_case - 1);
    end
end


subplot(3, 1, 1)
semilogy(1 : num_cases,  cond_conc_reg_mat); grid on;
title('Reg mat cond num');

subplot(3, 1, 2)
semilogy(1 : num_cases,  cond_lin_conc_reg_mat); grid on;
title('Lin mtum reg mat cond num');

subplot(3, 1, 3)
semilogy(1 : num_cases,  cond_ang_conc_reg_mat); grid on;
title('Ang mtum reg mat cond num');