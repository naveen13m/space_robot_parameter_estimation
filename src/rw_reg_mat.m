clear all; close all; clc;

% Read dynamic parameters for verification
base_sensor_base_frame_position_base_frame = [-0.2; -0.3; 0];
curr_dir = pwd;
cd(strcat('../test_case_data', '/2_link_rw', '/config_files'));
[num_links, not_planar, joint_twist_angle, link_length_DH, ...
    joint_offset, parent_link_index, link_x_com, link_y_com, link_z_com, ...
    ~, ~, link_mass, ~, inertia_xx_com, inertia_yy_com, inertia_zz_com, ...
    inertia_xy_com, inertia_yz_com, inertia_zx_com] = inputs();
link_x_com(1) = -base_sensor_base_frame_position_base_frame(1);
link_y_com(1) = -base_sensor_base_frame_position_base_frame(2);
link_z_com(1) = -base_sensor_base_frame_position_base_frame(3);
cd(curr_dir);

% Space robot locked
curr_mat = load('global_kin_mat_1.mat');
curr_mat = remove_excess(curr_mat.global_kin_mat_1);
reg_mat = curr_mat(:, 1 : 4);
rar = [link_x_com(2); link_y_com(2)];
x_r = [inertia_zz_com(2); link_mass(2); link_mass(2) * rar];

a00 = [link_x_com(1); link_y_com(1)];


b = -curr_mat(:, 5 : 8) * x_r;
x_computed = pinv(reg_mat) * b
mtum_error_computed = reg_mat * x_computed - b;

x_actual = [1469; link_mass(1) + link_mass(3); [260; 315]];
actual_computed = [x_actual, x_computed]
error_params = x_actual - x_computed

mtum_error_actual = reg_mat * x_actual - b;


subplot(1, 3, 1)
plot(mtum_error_actual(1 : 3 : end), 'g');
hold on; title('Lin x');
plot(mtum_error_computed(1 : 3 : end), 'r--');
subplot(1, 3, 2)
plot(mtum_error_actual(2 : 3 : end), 'g');
hold on; title('Lin y');
plot(mtum_error_computed(2 : 3 : end), 'r--');
subplot(1, 3, 3)
plot(mtum_error_actual(3 : 3 : end), 'g');
hold on; title('Ang x');
plot(mtum_error_computed(3 : 3 : end), 'r--');