function [reg_mat, out_vec, extended_param_vec] = run_me(robot_make, ...
                                base_sensor_base_frame_position_base_frame)
    close all; clc;

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

    % Load robot structural data
    cd(strcat('../test_case_data', robot_make, '/config_files'));
    [num_links, not_planar] = inputs();
    is_planar = 1 - not_planar;
    cd(curr_dir);
    
    global_kin_mat = global_kinematic_matrix(robot_make, ...
        base_sensor_base_frame_position_base_frame, statevar, mtvar);
    [reg_mat, out_vec] = reduce_gkm(global_kin_mat, is_planar);
    minimal_param_vec = pinv(reg_mat) * out_vec;
    extended_param_vec = extend_minimal_param_vec(minimal_param_vec, num_links, is_planar);
                            
    % Verification
    cd(strcat('../test_case_data', robot_make, '/config_files'));
    [num_links, not_planar, joint_twist, link_length_DH, ...
        joint_offset, parent_link_index, link_x_com, link_y_com, link_z_com, ...
        ~, ~, link_mass, ~, inertia_xx_com, inertia_yy_com, inertia_zz_com, ...
        inertia_xy_com, inertia_yz_com, inertia_zx_com] = inputs();
    link_x_com(1) = -base_sensor_base_frame_position_base_frame(1);
    link_y_com(1) = -base_sensor_base_frame_position_base_frame(2);
    link_z_com(1) = -base_sensor_base_frame_position_base_frame(3);
    cd(curr_dir);
    param_vec = construct_dyn_param_vector(...
        inertia_xx_com, inertia_yy_com, inertia_zz_com, inertia_xy_com, inertia_yz_com,...
        inertia_zx_com, link_mass, link_x_com, link_y_com, link_z_com, num_links);
    actual_minimal_param_vec = compute_minimal_param_vector(param_vec, ...
                                num_links, not_planar, joint_twist, link_length_DH, ...
                                joint_offset, parent_link_index, base_sensor_base_frame_position_base_frame);
%     mtum = reg_mat * actual_minimal_param_vec - out_vec;
    mtum = global_kin_mat * extended_param_vec;
    figure;
    subplot(2, 3, 1);
    plot(mtum(1 : 6 : end)); grid on; title('Px_{scaled}');
    subplot(2, 3, 2);
    plot(mtum(2 : 6 : end)); grid on; title('Py_{scaled}');
    subplot(2, 3, 3);
    plot(mtum(3 : 6 : end)); grid on; title('Pz_{scaled}');
    subplot(2, 3, 4);
    plot(mtum(4 : 6 : end)); grid on; title('Lx_{scaled}');
    subplot(2, 3, 5);
    plot(mtum(5 : 6 : end)); grid on; title('Ly_{scaled}');
    subplot(2, 3, 6);
    plot(mtum(6 : 6 : end)); grid on; title('Lz_{scaled}');
end