clear all; 
close all;
clc;

% Configuration parameters
% robot_make = '/2_link';
% robot_make = '/4_link';
robot_make = '/dual_arm';
base_sensor_position_base_frame = [-0.2; -0.3; 0];
sim_data = 1;
data_dir = {'/experimental_data', '/sim_real_data'};
real_data_filename = {'/statevar.dat', '/timevar.dat', '/mtvar.dat'};
experimental_data_filename = {'/statevar.dat', '/timevar.dat'};
data_filename = [{experimental_data_filename}, {real_data_filename}];

% Load data to workspace
curr_dir = pwd;
data_filename = data_filename{sim_data  + 1};
num_data_files = length(data_filename);
for i = 1 : num_data_files
    load(strcat(curr_dir(1:end-4), '/test_case_data', ...
        robot_make, data_dir{sim_data + 1}, data_filename{i}));
end

% Load robot structural and dynamic parameter data
curr_dir = pwd;
cd(strcat('../test_case_data', robot_make, '/config_files'));
[num_links_with_base, not_planar, ~, link_length_DH, ~, parent_link_index, ...
    link_x_com, link_y_com, link_z_com, link_length, ~, link_mass] = ...
    inputs();
cd(curr_dir);

% dbstop in verify_ang_mtum_model.m at 265

% Populate the linear momentum regressor matrix
% lin_mtum_reg_mat = linear_momentum_regressor_matrix(robot_make, ...
%     base_sensor_position_base_frame, statevar, mtvar);
verify_ang_mtum_model(robot_make,...
                     base_sensor_position_base_frame, statevar, mtvar);

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

% Populate angular momentum regressor matrix

% Compute inertia parameters



