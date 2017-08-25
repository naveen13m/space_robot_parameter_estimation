clear all; close all; clc;

% dbstop in compute_coeff_error.m at 60
num_cases = 1;
Ka1_error{num_cases} = 0;
Ka2_error{num_cases} = 0;
Ka3_error{num_cases} = 0;
curr_dir = pwd;

for curr_case_index = 1 : num_cases
    case_name = strcat('case', int2str(curr_case_index));
    cd(strcat('./global_kinematic_matrix'));
    gkm_filename = strcat(case_name, '.mat');
    load(gkm_filename);
    cd(curr_dir);
    [a, b, c] = compute_coeff_error(case_name, global_kin_mat);
    Ka1_error{curr_case_index} = a;
    Ka2_error{curr_case_index} = b;
    Ka3_error{curr_case_index} = c;
end