clear all; close all; clc;

base_sensor_base_frame_position_base_frame = [-0.2; -0.3; 0];
num_instants = 10;
num_equs_per_exp = num_instants * 3;
num_experiments = 3;
num_cols_per_body = 3;
num_joints = 2;
num_bodies = 2 * num_joints + 1;
reg_mat = zeros(num_equs_per_exp * num_experiments, 1 + (num_bodies * num_cols_per_body));

% No joint locked
curr_mat = load('global_kin_mat_1.mat');
curr_mat = remove_excess(curr_mat.global_kin_mat_1);
reg_mat(1 : num_equs_per_exp, 1 : 4) = [curr_mat(:, 2), curr_mat(:, 1), curr_mat(:, 3 : 4)];
reg_mat(1 : num_equs_per_exp, 8 : 10) = curr_mat(:, [5, 7 : 8]);
reg_mat(1 : num_equs_per_exp, 14 : 16) = curr_mat(:, [9, 11 : 12]);

% Joint - 1 locked
curr_mat = load('global_kin_mat_2.mat');
curr_mat = remove_excess(curr_mat.global_kin_mat_1);
reg_mat(num_equs_per_exp + 1: 2 * num_equs_per_exp, 1) = curr_mat(:, 2);
reg_mat(num_equs_per_exp + 1: 2 * num_equs_per_exp, 5 : 7) = curr_mat(:, [1, 3 : 4]);
reg_mat(num_equs_per_exp + 1: 2 * num_equs_per_exp, 14 : 16) = curr_mat(:, [9, 11 : 12]);

% Joint - 2 locked
curr_mat = load('global_kin_mat_3.mat');
curr_mat = remove_excess(curr_mat.global_kin_mat_1);
reg_mat(2 * num_equs_per_exp + 1 : 3 * num_equs_per_exp, 1 : 4) = [curr_mat(:, 2), curr_mat(:, 1), curr_mat(:, 3 : 4)];
reg_mat(2 * num_equs_per_exp + 1 : 3 * num_equs_per_exp, 11 : 13) = curr_mat(:, [5, 7 : 8]);


rank_reg_mat = rank(reg_mat);
coupled_params = pinv(reg_mat(:, 2 : end)) * (-1200 * reg_mat(:, 1));

curr_dir = pwd;
cd(strcat('../test_case_data', '/3_link', '/config_files'));
[num_links, not_planar, joint_twist_angle, link_length_DH, ...
    joint_offset, parent_link_index, link_x_com, link_y_com, link_z_com, ...
    ~, ~, link_mass, ~, inertia_xx_com, inertia_yy_com, inertia_zz_com, ...
    inertia_xy_com, inertia_yz_com, inertia_zx_com] = inputs();
link_x_com(1) = -base_sensor_base_frame_position_base_frame(1);
link_y_com(1) = -base_sensor_base_frame_position_base_frame(2);
link_z_com(1) = -base_sensor_base_frame_position_base_frame(3);
cd(curr_dir);

% Simple link lengths
l0 = [0.7; 0.3];
l1 = [1; 0];
l2 = [1; 0];
l0sq = l0.' * l0;
l1sq = l1.' * l1;

% Compound link lengths
th1 = 0;
R1 = rotz(th1);
R1(:, 3) = [];
R1(3, :) = [];

th2 = 0;
R2 = rotz(th2);
R2(:, 3) = [];
R2(3, :) = [];
l01 = l0 + R1 * l1;
l01sq = l01.' * l01;

coupled_params = [sum(link_mass); coupled_params];

m012 = sum(link_mass);
m12 = sum(link_mass(2 : 3));
m01 = sum(link_mass(1 : 2));
m0 = link_mass(1);
m1 = link_mass(2);
m2 = link_mass(3);
a0 = [link_x_com(1); link_y_com(1)];
a1 = [link_x_com(2); link_y_com(2)];
a2 = [link_x_com(3); link_y_com(3)];
I330 = inertia_zz_com(1) + m0 * (a0.' * a0);
I331 = inertia_zz_com(2) + m1 * (a1.' * a1);
I332 = inertia_zz_com(3) + m2 * (a2.' * a2);
I3301 = I330 + I331 + m1 * l0sq + 2 * l0.' * m1 * a1;
I3312 = I331 + I332 + m2 * l1sq + 2 * l1.' * m2 * a2;
a01 = (m0 * a0 + m1 * (l0 + R1 * a1)) / m01;
a12 = (m1 * a1 + m2 * (l1 + R2 * a2)) / m12;
                    
actual_coupled_params =  [m012;
                          I330 + m12 * l0sq;
                          m0 * a0 + m12 * l0;
                          I3301 + m2 * l01sq;
                          m01 * a01 + m12 * l01;
                          I331 + m2 * l1sq;
                          m1 * a1 + m2 * l1;
                          I3312
                          m12 * a12;
                          I332
                          m2 * a2];
                      
actual_and_computed_coupled = [actual_coupled_params, coupled_params]                      
                                                                                              
z12 = zeros(1, 2);
z21 = zeros(2, 1);
i22 = eye(2);
z22 = zeros(2, 2);

%     | Link - 0 | |   Link - 1     |     |  Link - 2  |
pcm = [0   1   z12 0   1         z12      0   1     z12;      % Link-0
       1   0   z12 0   l0sq      z12      0   l0sq  z12;      % 
       z21 z21 i22 z21 l0        z22      z21 l0    z22;      %
       % Line-1
       1   0   z12 1   l0sq      2 * l0.' 0   l01sq z12;      % Link-01
       z21 z21 i22 z21 l0 + l01  R1       z21 l01   z22;      %
       % Line-2
       0   0   z12 1   0         z12      0   l1sq  z12;      % Link-1
       z21 z21 z22 z21 z21       i22      z21 l1    z22;      %
       % Line-3
       0   0   z12 1   0         z12      1   l1sq  2 * l1.'; % Link-12
       z21 z21 z22 z21 z21       i22      z21 l1    R2;       %
       % Line-4
       0   0   z12 0   0         z12      1   0     z12;      % Link-2
       z21 z21 z22 z21 z21       z22      z21 z21   i22];     %
   
rref_pcm = rref(pcm);
rank_pcm = rank(pcm);
cond(pcm);
actual_params = pinv(pcm) * actual_coupled_params;
comp_params = pinv(pcm) * coupled_params;

%actual_and_computed_params = [actual_params, comp_params]

pcm_lin_mtum = pcm;
pcm_lin_mtum([1,5,8,11,14], :) = [];
rref(pcm_lin_mtum);
rank(pcm_lin_mtum);
