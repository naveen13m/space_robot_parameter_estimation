clear all; close all; clc;

reg_mat_filename = 'reg_mat_case';
out_vec_filename = 'output_vec_case';
num_cases = 2;

conc_reg_mat = [];
conc_out_vec = [];
for curr_case = 1 : num_cases
%     if curr_case ~= [6]
    if curr_case ~= [10]
        reg_mat{curr_case} = load(strcat(reg_mat_filename, int2str(curr_case), '.mat'));
        out_vec{curr_case} = load(strcat(out_vec_filename, int2str(curr_case), '.mat'));
        cond_reg_mat_case(curr_case) = cond(reg_mat{curr_case}.reg_mat_case);
        curr_reg_mat = reg_mat{curr_case}.reg_mat_case;
        curr_out_vec = out_vec{curr_case}.output_vec_case;
        check = rref(curr_reg_mat, 10e-3);
        coeffs(:, curr_case) = check(1 : 6, 7);
        conc_reg_mat = [conc_reg_mat; curr_reg_mat];
        conc_out_vec = [conc_out_vec; curr_out_vec];
        cond_conc_reg_mat(curr_case) = cond(conc_reg_mat);
        cond_lin_conc_reg_mat(curr_case) = cond(conc_reg_mat([1 : 3 : end, 2 : 3 : end], [2, 3, 5, 6, 8, 9]));
        cond_ang_conc_reg_mat(curr_case) = cond(conc_reg_mat(3 : 3 : end, [1 4 7]));
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

param_vec = [88.61,520,10,0,14.55,15,0,4,5,0].';
mtum = conc_reg_mat * param_vec([1, 3 : end]) - param_vec(2) * conc_out_vec;
figure;
subplot(3, 1, 1);
plot(mtum(1 : 3 : end, :))

subplot(3, 1, 2);
plot(mtum(2 : 3 : end, :))

subplot(3, 1, 3);
plot(mtum(3 : 3 : end, :))