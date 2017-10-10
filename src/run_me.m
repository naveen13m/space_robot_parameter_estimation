    function [reg_mat, out_vec, global_kin_mat, mtvar_reshaped] = run_me(robot_make, ...
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
   
%     % Parameter coupling matrix verification
%     load param_coupling_mat.mat
%     error_minimal_coup_mat = param_coupling_mat * param_vec - actual_minimal_param_vec
                            
    % Load robot structural data
    cd(strcat('../test_case_data', robot_make, '/config_files'));
    [num_links, not_planar] = inputs();
    is_planar = 1 - not_planar;
    cd(curr_dir);
    rw_params = make_reaction_wheel();
    
    global_kin_mat = global_kinematic_matrix(robot_make, ...
        base_sensor_base_frame_position_base_frame, statevar, mtvar, 0);
    [reg_mat, out_vec] = reduce_gkm(global_kin_mat, is_planar, rw_params);
    mtvar_reshaped = reshape(mtvar.', numel(mtvar), 1);
    out_vec = out_vec + randn(size(out_vec));
    minimal_param_vec_unfiltered = pinv(reg_mat) * (out_vec);
    
    check_int = 208;
    minimal_param_vec_other_sol = pinv(reg_mat(1 : (check_int * 5 * 6), :)) * (out_vec(1 : (check_int * 5 * 6)));

    num_intervals = 256;
    num_datapoints_per_interval = 5 - 0;
    num_equs = 6;
    num_rows = num_datapoints_per_interval * num_equs; 
    
    cn = zeros(num_intervals, 1);
    for curr_interval = 1 : num_intervals
        cn(curr_interval) = cond(reg_mat(1 : curr_interval * num_rows, :));
    end
    
    figure();
    subplot(2, 1, 1)
    semilogy(cn, 'k'); 
    ylabel('Condition Number', 'FontSize', 13);
    xlabel('Interval Index', 'FontSize', 13);
    set(gca, 'FontSize', 12,  'FontName', 'times')
    subplot(2, 1, 2)
    semilogy(129 : 256, cn(129 : end), 'k'); 
    ylabel('Condition Number', 'FontSize', 13);
    xlabel('Interval Index', 'FontSize', 13);
    set(gca, 'FontSize', 12, 'FontName', 'times')
%     print('cond_num', '-deps', '-r300');
                            
    compare = [(1 : 52).', actual_minimal_param_vec, minimal_param_vec_unfiltered, minimal_param_vec_other_sol]
    save actual_computed_params.mat actual_minimal_param_vec minimal_param_vec_unfiltered param_vec
    param_error_unfiltered = minimal_param_vec_unfiltered - actual_minimal_param_vec;
    per_error_min_parm = [(1 : 52).', (param_error_unfiltered./ actual_minimal_param_vec) * 100];
    mean_error = (1/52) * sum(abs(minimal_param_vec_unfiltered ./ actual_minimal_param_vec));
    mtum = reg_mat * minimal_param_vec_unfiltered - out_vec;
    mtum1 = reg_mat * actual_minimal_param_vec - out_vec;
    
    figure;
    subplot(2, 3, 1);
    plot(mtum(1 : 6 : end), 'k'); hold on; 
    plot(mtum1(1 : 6 : end), 'g--'); grid on;
    subplot(2, 3, 2);
    plot(mtum(2 : 6 : end), 'k'); hold on; 
    plot(mtum1(2 : 6 : end), 'g--'); grid on;
    subplot(2, 3, 3);
    plot(mtum(3 : 6 : end), 'k'); hold on; 
    plot(mtum1(3 : 6 : end), 'g--'); grid on;
    subplot(2, 3, 4);
    plot(mtum(4 : 6 : end), 'k'); hold on; 
    plot(mtum1(4 : 6 : end), 'g--'); grid on;
    subplot(2, 3, 5);
    plot(mtum(5 : 6 : end), 'k'); hold on; 
    plot(mtum1(5 : 6 : end), 'g--'); grid on;
    subplot(2, 3, 6);
    plot(mtum(6 : 6 : end), 'k'); hold on; 
    plot(mtum1(6 : 6 : end), 'g--'); grid on;
%     close all
end