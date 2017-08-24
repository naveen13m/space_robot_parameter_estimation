clear all; close all; clc;

% Read dynamic parameters for verification
base_sensor_base_frame_position_base_frame = [-0.2; -0.3; 0];
curr_dir = pwd;
cd(strcat('../test_case_data', '/2_link_mrw', '/kin_mat/case_0'));
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
l0sq = l0.' * l0;
R01_case_eff = eye(2);

% MRW parameters
mr = link_mass(end);
ar = [link_x_com(end); link_y_com(end)];
I33r = inertia_zz_com(end) + mr * (ar.' * ar);
phi_r = [I33r; mr; mr * ar];

% Actual parameter calculation for verification
m01 = sum(link_mass(1 : end - 1));
me = m01;
m0 = link_mass(1);
m1 = link_mass(2);
a0 = [link_x_com(1); link_y_com(1)];
a1 = [link_x_com(2); link_y_com(2)];
ae = (m0 * a0 + m1 * (l0 + R01_case_eff * a1)) / me;
I330 = inertia_zz_com(1) + m0 * (a0.' * a0);
I331 = inertia_zz_com(2) + m1 * (a1.' * a1);
I33e = I330 + I331 + m1 * l0sq + 2 * l0.' * R01_case_eff * m1 * a1;
phi_0 = [I330; m0; m0 * a0];
phi_1 = [I331; m1; m1 * a1];

actual_eff_params = [I33e;
                     me;
                     me * ae];
                       
% Computation of effective parameters 
cd(strcat('../test_case_data', '/2_link_mrw', '/kin_mat'));
curr_mat = load('global_kin_mat_eff.mat');
cd(curr_dir)
curr_mat = remove_excess(curr_mat.global_kin_mat);
K0 = curr_mat(:, 1 : 4);
Kr = curr_mat(:, 9 : 12);
computed_eff_params = - pinv(K0) * Kr * phi_r;
actual_computed_eff_params = [actual_eff_params, computed_eff_params]

% Computation of coupled parameters                  
actual_coupled_params =  [I330 + m1 * l0sq;
                          m01;
                          m0 * a0 + m1 * l0;
                          I331;
                          m1 * a1];

cd(strcat('../test_case_data', '/2_link_mrw', '/kin_mat'));
curr_mat = load('global_kin_mat_0.mat');
cd(curr_dir);
curr_mat = remove_excess(curr_mat.global_kin_mat);
reg_mat = curr_mat(:, [1 : 5, 7, 8]);
Kr = curr_mat(:, 9 : 12);
computed_coupled_params = -pinv(reg_mat) * Kr * phi_r;
actual_computed_eff_params = [actual_coupled_params, computed_coupled_params]

% Computation of decoupled parameters
actual_decoupled_params = [I330;
                           m0;
                           m0 * a0;
                           I331;
                           m1;
                           m1 * a1];
                       
z12 = zeros(1, 2);
z21 = zeros(2, 1);
i22 = eye(2);
z22 = zeros(2, 2);

% pcm = [1   0     z12      0   l0sq  z12;
%        0   1     z12      0   1     z12;
%        z21 z21   i22      z21 l0    z22;
%        0   0     z12      1   0     z12;
%        z21 z21   z22      z21 z21   i22;
%        1   0     z12      1   l0sq  2 * l0.' * R01_case_eff;
%        z21 z21   i22      z21 l0    R01_case_eff];

% rref_pcm = rref(pcm)
% rank_pcm = rank(pcm)
% 
% computed_decoupled_params = pinv(pcm) * [computed_coupled_params; computed_eff_params(1); computed_eff_params(3 : 4)];
% actual_computed_decoupled = [actual_decoupled_params, computed_decoupled_params]

       
% Attempt
aesq = ae.' * ae;
k0 = [0.5; 0.3] - ae;
k0sq = k0.' * k0;

pcm = [1   0     z12       0   l0sq  z12;
       0   1     z12       0   1     z12;
       z21 z21   i22       z21 l0    z22;
       0   0     z12       1   0     z12;
       z21 z21   z22       z21 z21   i22;
       1   aesq  -2 * ae.' 1   k0sq  2 * k0.' * R01_case_eff];
% 
rref_pcm = rref(pcm)
rank_pcm = rank(pcm)

computed_decoupled_params = pinv(pcm) * [computed_coupled_params; computed_eff_params(1)];
actual_computed_decoupled = [actual_decoupled_params, computed_decoupled_params]
          

% troubleshoot
% backward_computed_coupled = pcm * actual_decoupled_params
% backward_computed_coupled1 = pcm * computed_decoupled_params