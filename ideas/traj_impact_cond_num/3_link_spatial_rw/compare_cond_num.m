clear all; close all; clc;

% Config params
base_sensor_base_frame_position_base_frame = [-0.2; -0.3; -0.4];
filename1 = 'global_kin_mat';
filename2 = 'mtvar';
num_cases = 4;

conc_reg_mat = [];
conc_out_vec = [];
cutt_off = 0;
rw_param = [1; 1; 2; 0; 0; 0; 20; 0; 0; 0];
rw_params = [rw_param; rw_param; zeros(10, 1); rw_param];
rw_params1 = make_reaction_wheel
mtvar_all = [];
for curr_case = 1 : num_cases
    if curr_case ~= [10]
%     if curr_case ~= [4, 5, 6]
        reg_mat{curr_case} = load(strcat(filename1, int2str(curr_case), '.mat'));
        mtvar = load(strcat(filename2, int2str(curr_case), '.mat'));
        mtvar_all = [mtvar_all; mtvar.mtvar];
        cond_reg_mat_case(curr_case) = cond(reg_mat{curr_case}.global_kin_mat);
        curr_reg_mat = reg_mat{curr_case}.global_kin_mat;
        out_vec_case = -curr_reg_mat(cutt_off + 1: end - cutt_off, 31 : 70) * rw_params;
        curr_reg_mat(:, [12, 17, 20, 22, 27, 30, 31 : 70]) = [];
        [curr_reg_mat1, out_vec_case1] = reduce_gkm(reg_mat{curr_case}.global_kin_mat, 0, rw_params1);
        curr_reg_mat = curr_reg_mat(cutt_off + 1: end - cutt_off, :);
        check(:, :, curr_case) = rref(curr_reg_mat, 10e-3);
        conc_reg_mat = [conc_reg_mat; curr_reg_mat];
        conc_out_vec = [conc_out_vec; out_vec_case];
        cond_conc_reg_mat(curr_case) = cond(conc_reg_mat);
    else
        cond_conc_reg_mat(curr_case) = cond_conc_reg_mat(curr_case - 1);
    end
end

% subplot(3, 1, 1)
% semilogy(1 : num_cases,  cond_conc_reg_mat); grid on;
% title('Reg mat cond num');

minimal_param_vec = pinv(conc_reg_mat) *  conc_out_vec;

% verification
[num_links, not_planar, joint_twist, link_length_DH, ...
    joint_offset, parent_link_index, link_x_com, link_y_com, link_z_com, ...
    ~, ~, link_mass, ~, inertia_xx_com, inertia_yy_com, inertia_zz_com, ...
    inertia_xy_com, inertia_yz_com, inertia_zx_com] = inputs();
link_x_com(1) = -base_sensor_base_frame_position_base_frame(1);
link_y_com(1) = -base_sensor_base_frame_position_base_frame(2);
link_z_com(1) = -base_sensor_base_frame_position_base_frame(3);
param_vec = construct_dyn_param_vector(...
        inertia_xx_com, inertia_yy_com, inertia_zz_com, inertia_xy_com, inertia_yz_com,...
        inertia_zx_com, link_mass, link_x_com, link_y_com, link_z_com, num_links);
actual_minimal_param_vec = compute_minimal_param_vector(param_vec, ...
    num_links, not_planar, joint_twist, link_length_DH, ...
    joint_offset, parent_link_index, base_sensor_base_frame_position_base_frame);
mtum = conc_reg_mat * minimal_param_vec - conc_out_vec;
compare = [actual_minimal_param_vec, minimal_param_vec]

figure();
subplot(2, 3, 1);
plot(mtum(1 : 6 : end));
hold on; plot(mtvar_all(1 : 6 : end));
plot(mtum(1 : 6 : end) - mtvar_all(1 : 6 : end));

subplot(2, 3, 2);
plot(mtum(2 : 6 : end)); 
hold on; plot(mtvar_all(2 : 6 : end));
plot(mtum(2 : 6 : end) - mtvar_all(2 : 6 : end));

subplot(2, 3, 3);
plot(mtum(3 : 6 : end)); 
hold on; plot(mtvar_all(3 : 6 : end));
plot(mtum(3 : 6 : end) - mtvar_all(3 : 6 : end));

subplot(2, 3, 4);
plot(mtum(4 : 6 : end)); 
hold on; plot(mtvar_all(4 : 6 : end));
plot(mtum(4 : 6 : end) - mtvar_all(4 : 6 : end));

subplot(2, 3, 5);
plot(mtum(5 : 6 : end)); 
hold on; plot(mtvar_all(5 : 6 : end));
plot(mtum(5 : 6 : end) - mtvar_all(5 : 6 : end));

subplot(2, 3, 6);
plot(mtum(6 : 6 : end)); 
hold on; plot(mtvar_all(6 : 6 : end));
plot(mtum(6 : 6 : end) - mtvar_all(6 : 6 : end));
legend('estimated', 'observed', 'error')

cond(conc_reg_mat)