clear all; 
close all;
clc;

% Configuration parameters
robot_make = '/dual_arm';
base_sensor_position_base_frame = [0; 0; 0];
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

dbstop in linear_momentum_regressor_matrix.m at 102
%% Populate the linear momentum regressor matrix
lin_mtum_reg_mat = linear_momentum_regressor_matrix(robot_make, ...
    base_sensor_position_base_frame, statevar);

% Compute mass and CoM parameters

% Populate angular momentum regressor matrix

% Compute inertia parameters



