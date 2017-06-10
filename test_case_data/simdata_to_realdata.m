function simdata_to_realdata(robot_make, base_sensor_position_base_frame)
    % Load simulation data
    curr_dir = pwd;
    data_filename = {'/statevar.dat', '/timevar.dat', '/mtvar.dat'};
    num_data_files = length(data_filename);
    for curr_instant = 1 : num_data_files
        load(strcat(curr_dir, robot_make, '/sim_data', ...
            data_filename{curr_instant}));
    end
    
    % Compute base sensor frame data based on the base simulation frame data
    len_time = length(timevar);
    cd(strcat(curr_dir, robot_make, '/config_files'));
    num_links = inputs();
    num_links = num_links - 1; % Excluding the base link
    cd(curr_dir);
    base_com_position = statevar(:, 1 : 3);
    base_com_orientation = statevar(:, 4 : 6);
    %joint_position = statvar(:, 7 : 6 + num_links);
    base_com_lin_vel = statevar(:, 7 + num_links : 9 + num_links);
    base_com_ang_vel = statevar(:, 10 + num_links : 12 + num_links);
    temp = base_com_ang_vel(:, 1);
    base_com_ang_vel(:, 1) = base_com_ang_vel(:, 2);
    base_com_ang_vel(:, 2) = base_com_ang_vel(:, 3);
    base_com_ang_vel(:, 3) = temp;
    %joint_velocity = statevar(:, 13 + num_links : 12 + 2 * num_links);
    base_sensor_position = zeros(len_time, 3);
    base_sensor_velocity = zeros(len_time, 3);
    for curr_instant = 1 : len_time
        phi = base_com_orientation(curr_instant);
        rot_mat_base = [cos(phi) -sin(phi) 0
                        sin(phi)  cos(phi) 0
                        0         0        1];
        base_sensor_position(curr_instant, :) = ...
        (base_com_position(curr_instant, 1:3).' + ...
            rot_mat_base*base_sensor_position_base_frame).';
        base_sensor_velocity(curr_instant, :) = ...
        (base_com_lin_vel(curr_instant, 1:3).' + ...
            cross(base_com_ang_vel(curr_instant, 1 : 3).', ...
            rot_mat_base * base_sensor_position_base_frame)).';         
    end
    
    % Add noise to kinematic data
    add_noise = 0;
    base_position_std_dev = 1;
    base_orientation_std_dev = 1;
    joint_postion_std_dev = 1;
    base_lin_vel_std_dev = 1;
    base_ang_vel_std_dev = 1;
    joint_velocity_std_dev = 1;
    std_dev = [base_position_std_dev*ones(1,3), ...
        base_orientation_std_dev*ones(1,3), ...
        joint_postion_std_dev*ones(1, num_links), ...
        base_lin_vel_std_dev*ones(1,3), ...
        base_ang_vel_std_dev*ones(1,3), ...
        joint_velocity_std_dev*ones(1, num_links)];
    num_states = 12 + 2 * num_links;
    noise(len_time, num_states) = 0;
    for curr_state = 1 : num_states
        noise(:, curr_state) = ...
            add_noise*std_dev(curr_state)*randn(len_time, 1);
    end
    statevar = statevar(:, 1:num_states) + noise;
    save(strcat(curr_dir, robot_make, '/sim_real_data', '/statevar.dat'),...
        'statevar', '-ascii');
end

