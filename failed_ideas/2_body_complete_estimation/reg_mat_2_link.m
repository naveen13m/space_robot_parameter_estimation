clear all; close all; clc;

% Read dynamic parameters for verification
base_sensor_base_frame_position_base_frame = [-0.2; -0.3; 0];
curr_dir = pwd;
cd(strcat('../test_case_data', '/2_link', '/config_files'));
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
h0 = [32/110; 33/110];
h1 = [32/110 - 0.7; 0];

l0sq = l0.' * l0;
h0sq = h0.' * h0;
h1sq = h1.' * h1;

m01 = sum(link_mass);
m0 = link_mass(1);
m1 = link_mass(2);
a0 = [link_x_com(1); link_y_com(1)];
a1 = [link_x_com(2); link_y_com(2)];
I330 = inertia_zz_com(1) + m0 * (a0.' * a0);
I331 = inertia_zz_com(2) + m1 * (a1.' * a1);
I_sys_cm = inertia_zz_com(1) + m0 * (32/110 - 0.2)^2 + ...
    inertia_zz_com(2) + m1 * (1.2 - 32/110)^2; 
I_sys_eff = I_sys_cm + 1100 * h0.' * h0;
actual_phi_0 = [I330; m0; m0 * a0];
actual_phi_1 = [I331; m1; m1 * a1];

actual_coupled_params =  [I330 + m1 * l0sq;
                          m01;
                          m0 * a0 + m1 * l0;
                          I331;
                          m1 * a1;
                          I_sys_cm];

actual_decoupled_params = [I330;
                           m0;
                           m0 * a0;
                           I331;
                           m1;
                           m1 * a1];

% No joint locked
cd(strcat('../test_case_data', '/2_link', '/kin_mat'));
curr_mat = load('global_kin_mat_0.mat');
cd(curr_dir)
curr_mat = remove_excess(curr_mat.global_kin_mat);
reg_mat = curr_mat;
reg_mat(:, [2, 6]) = [];
b = -m01 * curr_mat(:, 2);
computed_coupled_params = pinv(reg_mat) * b;
% computed_coupled_params = [computed_coupled_params(1); m01; computed_coupled_params(2 : end)];
computed_coupled_params = [computed_coupled_params(1); m01; computed_coupled_params(2 : end); I_sys_cm];%; 320; 330];
% actual_computed_coupled = [actual_coupled_params, computed_coupled_params]
                                                                                              
z12 = zeros(1, 2);
z21 = zeros(2, 1);
i22 = eye(2);
z22 = zeros(2, 2);

pcm = [1   0     z12      0   l0sq  z12;
       0   1     z12      0   1     z12; 
       z21 z21   i22      z21 l0    z22;
       0   0     z12      1   0     z12; 
       z21 z21   z22      z21 z21   i22; 
       1   h0sq -2 * h0.' 1   h1sq -2 * h1.'];
%        z21 z21   i22      z21 l0    i22];
       

          
% rref_pcm = rref(pcm)
% rank_pcm = rank(pcm)
% lb = [0, 950, -300, -300, 0, 0, 0, 0];
% ub = [2000, 2000, 300, 300, 200, 200, 100, 100];
% x0 = [10, 100, 10, 20, 100, 30, 49, 20];
% % options = optimoptions(@fmincon,'Algorithm','active-set');%, 'TolFun', 1e-10, 'FunValCheck', 'on', 'TolCon', 1e-10, 'TolX', 1e-10)
% options = optimoptions('particleswarm','SwarmSize',200,'HybridFcn',@patternsearch);
% [x,fval,exitflag,output] = particleswarm(@(x)(10^8 * norm((pcm * x.' - computed_coupled_params))), 8, lb, ub, options)

K0 = curr_mat(:, 1 : 4);
K1 = curr_mat(:, 5 : 8);
G = [1,  l0sq, -2 * l0.';
     0,   1,     z12;
     z21, -l0,   i22];
 
reg_mat = K0 - K1 * G;
% rank(reg_mat)
phi_eff = [I_sys_eff; 1100; 320; 330];
phi_0_red = -pinv(reg_mat(:, 1 : 3)) * K1 * G * phi_eff
% rhs = reg_mat * actual_phi_0 + K1 * G * phi_eff;
% plot(rhs)
check = rref(reg_mat);