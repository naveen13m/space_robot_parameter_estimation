clear all; close all; clc;

num_cases = 5;
filename = {'global_kin_mat_case1.mat', 'global_kin_mat_case2.mat', ...
    'global_kin_mat_case3.mat', 'global_kin_mat_case4-1.mat' ...
    'global_kin_mat_case4-2.mat'};
phi_r = {[10,  20, 0, 0], ...
         [50, 100, 0, 0], ...
         [50, 100, 0, 0], ...
         [10,  20, 0, 0, 10,  20, 0, 0], ...
         [10,  20, 0, 0]};
     
  
ld_coeffs = zeros(3, num_cases);       
coupled_base_params = zeros(3, num_cases);
for case_index = 1 : num_cases
    curr_mat = load(filename{case_index});
    curr_mat = remove_excess(curr_mat.global_kin_mat_1);
    K0 = curr_mat(:, 1 : 4);
    Kr = curr_mat(:, 5 : end);
    ld_coeffs(:, case_index) = pinv(K0(:, 1 : 3)) * K0(:, 4);
    coupled_base_params(:, case_index) = -pinv(K0(:, 1 : 3)) * Kr * phi_r{case_index}.';
end

ld_coeffs
coupled_base_params
reg_mat = [eye(3) ld_coeffs(:, 1);
           eye(3) ld_coeffs(:, 2)]

rhs = [coupled_base_params(:, 1);
       coupled_base_params(:, 2)]

base_params = pinv(reg_mat) * rhs

