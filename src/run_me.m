clear all; 
close all;
clc;

dbstop in regressor_matrix.m  at 213
% Configuration parameters
% robot_make = '/2_link';
% robot_make = '/3_link';
% robot_make = '/4_link';
% robot_make = '/8_link';
robot_make = '/dual_arm';
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
num_instants = size(statevar, 1);

% Populate the linear momentum regressor matrix
reg_mat = regressor_matrix(robot_make, ...
    base_sensor_position_base_frame, statevar, mtvar);
