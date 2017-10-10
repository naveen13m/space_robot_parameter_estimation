% Validates the obtained parameters by reconstructing the momentum applied
% by the reaction wheels on the space robot using a validation trajectory.
% The file needs the actual and computed minimal parameters to reconstruct
% the compare the true and predicted momentum.
function validate_params(robot_make, base_sensor_base_frame_position_base_frame)
    load actual_computed_params.mat;
    data_dir = '/sim_real_data';
    data_filename = {'/statevar.dat', '/timevar.dat', '/mtvar.dat'};
    % Load data to workspace
    curr_dir = pwd;
    num_data_files = length(data_filename);
    for i = 1 : num_data_files
        load(strcat( '../test_case_data', robot_make, data_dir,...
            data_filename{i}));
    end
    cd(curr_dir);
   

    % Load robot structural data
    cd(strcat('../test_case_data', robot_make, '/config_files'));
    [num_links, not_planar] = inputs();
    is_planar = 1 - not_planar;
    cd(curr_dir);
    rw_params = make_reaction_wheel();
    
    global_kin_mat = global_kinematic_matrix(robot_make, ...
        base_sensor_base_frame_position_base_frame, statevar, mtvar, 0);
    [reg_mat, out_vec] = reduce_gkm(global_kin_mat, is_planar, rw_params);
%     four_sol = pinv(reg_mat) * (out_vec + mtvar)
%     compare = [four_sol, actual_minimal_param_vec]
    actual_rw_mtum = reg_mat * actual_minimal_param_vec;
    prefilter_comp_rw_mtum = reg_mat * minimal_param_vec_unfiltered;
    
       
    out_vec = reshape(out_vec, 6, numel(out_vec)/6).' + mtvar;
    actual_rw_mtum = reshape(actual_rw_mtum, 6, numel(actual_rw_mtum)/6).';
    prefilter_comp_rw_mtum = reshape(prefilter_comp_rw_mtum, 6, numel(prefilter_comp_rw_mtum)/6).';
    
    error_prefilter = actual_rw_mtum  - prefilter_comp_rw_mtum;  
    
    close all;
    rms_error = zeros(1, 6);
    rms_actual = zeros(1, 6);
    y_label = {'p_x', 'p_y', 'p_z', 'l_x', 'l_y', 'l_z'};
    for i = 1 : 6
        figure(1);
        subplot(2, 3, i);
        plot(timevar, -actual_rw_mtum(:, i), 'k'); hold on;
        plot(timevar, -prefilter_comp_rw_mtum(:, i), 'm--');
%         figure(2);
%         subplot(2, 3, i);
        plot(timevar, -error_prefilter(:, i), 'r-.'); 
        rms_error(i) = rms(error_prefilter(:, i));
        rms_actual(i) = rms(actual_rw_mtum(:, i));
        xlabel('time [s]');
        ylabel(strcat(y_label(i), ' [Nms]'));
    end
    
    hl = legend('True', 'Computed', 'Error', 'Orientation', 'horizontal');
    set(hl,'Position', [0.50 0.465 0.04 0.04], 'Units', 'normalized');
    set(gcf,'units','pixels','position',[10, 0, 1366, 768]); % [x, y, width, height]
    rms_error
    rms_actual
    verification_params = rms_error ./ rms_actual
%     print('mtum_validation', '-depsc', '-r300');    
end

