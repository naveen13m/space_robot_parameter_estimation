    function [reg_mat, out_vec, global_kin_mat] = run_me(robot_make, ...
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
        load(strcat( '../test_case_data', robot_make, data_dir{sim_data + 1},...
            data_filename{i}));
    end

     % Verification
    cd(strcat('../test_case_data', robot_make, '/config_files'));
    [num_links, not_planar, joint_twist, link_length_DH, ...
        joint_offset, parent_link_index, link_x_com, link_y_com, link_z_com, ...
        ~, ~, link_mass, ~, inertia_xx_com, inertia_yy_com, inertia_zz_com, ...
        inertia_xy_com, inertia_yz_com, inertia_zx_com] = inputs_robot();
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
   

    % Load robot structural data
%     cd(strcat('../test_case_data', robot_make, '/config_files'));
%     [num_links, not_planar] = inputs();
    is_planar = 1 - not_planar;
%     cd(curr_dir);
    rw_params = make_reaction_wheel();
    
    global_kin_mat = global_kinematic_matrix(robot_make, ...
        base_sensor_base_frame_position_base_frame, statevar, mtvar);
    [reg_mat, out_vec] = reduce_gkm(global_kin_mat, is_planar, rw_params);


    minimal_param_vec = pinv(reg_mat) * (out_vec);

%     num_intervals = 64;
%     num_datapoints_per_interval = 10 - 0;
%     num_equs = 6;
%     num_rows = num_datapoints_per_interval * num_equs; 
%     
%     cn = zeros(num_intervals, 1);
%     for curr_interval = 1 : num_intervals
%         cn(curr_interval) = cond(reg_mat(1 : curr_interval * num_rows, :));
%     end
    
%     figure();
%     subplot(3, 1, 1)
%     semilogy(cn); grid on;
%     subplot(3, 1, 2)
%     plot(cn(1 : 8)); grid on;
%     subplot(3, 1, 3)
%     semilogy(cn(9 : end)); grid on;
    
                            
    compare = [actual_minimal_param_vec, minimal_param_vec]
    param_error = minimal_param_vec - actual_minimal_param_vec;
    per_error_min_parm = (param_error./ actual_minimal_param_vec) * 100
    mtum = reg_mat * minimal_param_vec - out_vec;
    mtum1 = reg_mat * actual_minimal_param_vec - out_vec;
    
    figure;
    subplot(2, 3, 1);
    plot(mtum(1 : 6 : end)); hold on; plot(mtum1(1 : 6 : end)); grid on; title('Px_{RW}');
    subplot(2, 3, 2);
    plot(mtum(2 : 6 : end)); hold on; plot(mtum1(2 : 6 : end)); grid on; title('Py_{RW}');
    subplot(2, 3, 3);
    plot(mtum(3 : 6 : end)); hold on; plot(mtum1(3 : 6 : end)); grid on; title('Pz_{RW}');
    subplot(2, 3, 4);
    plot(mtum(4 : 6 : end)); hold on; plot(mtum1(4 : 6 : end)); grid on; title('Lx_{RW}');
    subplot(2, 3, 5);
    plot(mtum(5 : 6 : end)); hold on; plot(mtum1(5 : 6 : end)); grid on; title('Ly_{RW}');
    subplot(2, 3, 6);
    plot(mtum(6 : 6 : end)); hold on; plot(mtum1(6 : 6 : end)); grid on; title('Lz_{RW}');
% %     close all
end