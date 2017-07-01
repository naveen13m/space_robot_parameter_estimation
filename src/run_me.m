clear all; 
close all;
clc;

% dbstop in regressor_matrix.m  at 293
% Configuration parameters
% robot_make = '/2_link';
% robot_make = '/3_link';
% robot_make = '/4_link';
robot_make = '/8_link';
% robot_make = '/dual_arm';
% robot_make = '/2_link_spatial';
% robot_make = '/4_link_spatial';
% robot_make = '/temp_system';
% robot_make = '/static_base';

base_sensor_position_base_frame = [-0.2; -0.3; 0];
sim_data = 1;
data_dir = {'/experimental_data', '/sim_real_data'};
experimental_data_filename = {'/statevar.dat', '/timevar.dat'};
real_data_filename = {'/statevar.dat', '/timevar.dat', '/mtvar.dat'};
data_filename = [{experimental_data_filename}, {real_data_filename}];

% Load data to workspace
curr_dir = pwd;
data_filename = data_filename{sim_data  + 1};
num_data_files = length(data_filename);
for i = 1 : num_data_files
    load(strcat(curr_dir(1 : end - 4), '/test_case_data', ...
        robot_make, data_dir{sim_data + 1}, data_filename{i}));
end

% Load robot structural and dynamic parameter data
curr_dir = pwd;
cd(strcat('../test_case_data', robot_make, '/config_files'));
[num_links_with_base, not_planar, ~, link_length_DH, ~, parent_link_index, ...
    link_x_com, link_y_com, link_z_com, link_length, ~, link_mass] = ...
    inputs();
cd(curr_dir);

% Populate the linear momentum regressor matrix
reg_mat = regressor_matrix(robot_make, ...
    base_sensor_position_base_frame, statevar, mtvar);
% rref_reg_mat = rref(reg_mat);
% rank_reg_mat = rank(reg_mat)
% red_reg_mat = reg_mat;
% red_reg_mat(:, 17) = red_reg_mat(:, 17) - red_reg_mat(:, 7);
% red_reg_mat(:, 27) = red_reg_mat(:, 27) - red_reg_mat(:, 7);
% red_reg_mat(:, [1, 2, 4, 7, 11, 12, 14, 21, 22, 24]) = [];
% rref_red_reg_mat = rref(red_reg_mat);
% rank_red_reg_mat = rank(red_reg_mat)

% Compute mass and CoM parameters
% num_instants = length(timevar);
% lin_mtum(1 : 2 : 2 * num_instants, :) = mtvar(:, 1);
% lin_mtum(2 : 2 : 2 * num_instants, :) = mtvar(:, 2);
% joint_lin_velocity = lin_mtum_reg_mat(:, 4);
% reduced_lin_mtum_reg_mat = lin_mtum_reg_mat;
% 
% reduced_lin_mtum_reg_mat(:, 4 : 3 : end) = [];
% rhs_with_noisy_mtum = lin_mtum - (lin_mtum_reg_mat(:, 4 : 3 : end) * link_mass(2 : end));
% rhs_without_noisy_mtum =  - (lin_mtum_reg_mat(:, 4 : 3 : end) * link_mass(2 : end));
% params_with_noise = pinv(reduced_lin_mtum_reg_mat) * rhs_with_noisy_mtum
% params_without_noise = pinv(reduced_lin_mtum_reg_mat) * rhs_without_noisy_mtum
% cond_no = cond(reduced_lin_mtum_reg_mat)



