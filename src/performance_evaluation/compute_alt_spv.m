clear all; close all; clc;

load('actual_computed_params.mat');
load('param_coupling_mat.mat');

% minimal_param_vec = actual_minimal_param_vec;
minimal_param_vec = minimal_param_vec_unfiltered;

redundant_param_index = [8, 9, 10];
num_redundant_params = length(redundant_param_index);
redundant_param_value = param_vec(redundant_param_index);

alt_std_param_vec = minimal_to_std_param_vec(param_coupling_mat, minimal_param_vec, ...
                              redundant_param_value, redundant_param_index);
                          
verification_error = param_coupling_mat * alt_std_param_vec -  minimal_param_vec;

input_param_vec = spv_to_inertial_param(alt_std_param_vec);
save input_param_vec.mat input_param_vec