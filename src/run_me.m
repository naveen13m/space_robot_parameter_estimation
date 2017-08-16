clear all; 
close all;
clc;

% dbstop in regressor_matrix.m  at 213
% Configuration parameters
robot_make = '/1_link_rw';
% robot_make = '/1_link_2rw';
% robot_make = '/2_link';
% robot_make = '/3_link';
% robot_make = '/4_link';
% robot_make = '/8_link';
% robot_make = '/dual_arm';
% robot_make = '/2_link_rw';
% robot_make = '/4_link_spatial';
% robot_make = '/temp_system';
% robot_make = '/static_base';

base_sensor_position_base_frame = [0; 0; 0];
% base_sensor_position_base_frame = [-0.2; -0.3; 0];
% base_sensor_position_base_frame = [0.1; 0.2; 0];
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
[num_links_with_base, not_planar, joint_twist_angle, link_length_DH, ~, parent_link_index, ...
    link_x_com, link_y_com, link_z_com, link_length, ~, link_mass] = ...
    inputs();
cd(curr_dir);
num_instants = size(statevar, 1);

% Populate the linear momentum regressor matrix
global_kin_mat_1 = global_kinematic_matrix(robot_make, ...
    base_sensor_position_base_frame, statevar, mtvar);
reg_mat = remove_excess(global_kin_mat_1);
rref_reg_mat = rref(reg_mat, 10^-5);
rref_global_kin_mat = rref(global_kin_mat_1);

% b_actual = -10 * reg_mat(:, 5) - 50 * reg_mat(:, 6);
% A = reg_mat(:, 1 : 3);
% x_comp = pinv(A) * b_actual;
% % x_actual = [1330 - 0.466969034720210 * 300; 1000 - 3.333334097042634 * 300; 200 - 0.746032174302360 * 300]; % 0.5
% x_actual = [1330 - 0.504384007488360 * 300; 1000 - 3.333334108112251 * 300; 200 - 0.825397163698678 * 300]; % 1 
% % x_actual = [1330 - 0.500826351786586 * 300; 1000 - 3.333335493225453 * 300; 200 - 0.818182372089360 * 300]; % 0.5, diff mtum wheel dyna params
% % x_actual = [1330 - 0.567592476990328 * 300; 1000 - 3.333333895966867 * 300; 200 - 0.944444794646752 * 300]; %0.5 different base params
% x_actual_comp_error = [x_actual, x_comp, x_actual - x_comp]
% 
% b_comp = A * x_actual;
% rhs_actual_comp_error = [b_actual, b_comp, b_actual - b_comp];
% 
% mtum_actual = A * x_actual - b_actual;
% mtum_comp = A * x_comp - b_actual;
% 
% figure(3);
% subplot(2, 3, 1);
% plot(b_actual(1 : 3 : end), 'g'); hold on;
% plot(b_comp(1 : 3 : end), 'r--')
% 
% subplot(2, 3, 2);
% plot(b_actual(2 : 3 : end), 'g'); hold on;
% plot(b_comp(2 : 3 : end), 'r--')
% 
% subplot(2, 3, 3);
% plot(b_actual(3 : 3 : end), 'g'); hold on;
% plot(b_comp(3 : 3 : end), 'r--')
% 
% subplot(2, 3, 4);
% plot(mtum_actual(1 : 3 : end), 'g'); hold on;
% plot(mtum_comp(1 : 3 : end), 'r--')
% 
% subplot(2, 3, 5);
% plot(mtum_actual(2 : 3 : end), 'g'); hold on;
% plot(mtum_comp(2 : 3 : end), 'r--')
% 
% subplot(2, 3, 6);
% plot(mtum_actual(3 : 3 : end), 'g'); hold on;
% plot(mtum_comp(3 : 3 : end), 'r--')


% actual_coeffs = rref_global_kin_mat(1 : 6, 20)
% l = [0.5; 0; 0] - base_sensor_position_base_frame
% al = joint_twist_angle(2)
% computed_coeffs = [2 * l(3) * cos(al) - 2 * l(2) * sin(al);
%                    2 * l(3) * cos(al); 
%                    -2 * l(2) * sin(al);
%                    l(1) * sin(al);
%                    l(3) * sin(al) - l(2) * cos(al);
%                    - l(1) * cos(al)]
                    
% reg_mat(601, :) = [1, 0, 0, 1, 0, 0];
% rank(reg_mat)
% cond(reg_mat)
% reg_mat(:, 4) = [];
% null(reg_mat)
% cond(reg_mat(:, 2:5))
% sol = -1100 * pinv(reg_mat(:, 2 : 5)) * reg_mat(:, 1)
