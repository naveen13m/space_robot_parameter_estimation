clear all; close all; clc;

filename = 'global_kin_mat';
num_cases = 4;

conc_reg_mat = [];
conc_out_vec = [];
for curr_case = 1 : num_cases
    if curr_case ~= [10]
%     if curr_case ~= [4, 5, 6]
        reg_mat{curr_case} = load(strcat(filename, int2str(curr_case), '.mat'));
        cond_reg_mat_case(curr_case) = cond(reg_mat{curr_case}.global_kin_mat);
        curr_reg_mat = reg_mat{curr_case}.global_kin_mat;
        out_vec_case = -curr_reg_mat(:, [33, 37, 38, 39]) * [10; 20; 0; 0];
        curr_reg_mat = curr_reg_mat(:, [3, 7, 8, 9, 13, 18, 19, 23, 28, 29]);
        check(:, :, curr_case) = rref(curr_reg_mat, 10e-3);
        conc_reg_mat = [conc_reg_mat; curr_reg_mat(:, 1 : 10)];
        conc_out_vec = [conc_out_vec; out_vec_case];
        cond_conc_reg_mat(curr_case) = cond(conc_reg_mat);
    else
        cond_conc_reg_mat(curr_case) = cond_conc_reg_mat(curr_case - 1);
    end
end


subplot(3, 1, 1)
semilogy(1 : num_cases,  cond_conc_reg_mat); grid on;
title('Reg mat cond num');

params = pinv(conc_reg_mat) * conc_out_vec