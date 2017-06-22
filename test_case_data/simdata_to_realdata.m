function simdata_to_realdata(robot_make, base_sensor_position_com_frame)
    % Load simulation data
    close all;
    curr_dir = pwd;
    data_filename = {'/statevar.dat', '/timevar.dat', '/mtvar.dat'};
    num_data_files = length(data_filename);
    for curr_instant = 1 : num_data_files
        load(strcat(curr_dir, robot_make, '/sim_data', data_filename{curr_instant}));
    end
    
    % Compute base sensor data in the modified inertial frame
    len_time = length(timevar);
    cd(strcat(curr_dir, robot_make, '/config_files'));
    num_links_with_base = inputs();
    num_links_without_base = num_links_with_base - 1;
    cd(curr_dir);
    base_com_position = statevar(:, 1 : 3);
    base_com_orientation = statevar(:, 4 : 6);
    %joint_position = statvar(:, 7 : 6 + num_links);
    base_com_lin_vel = statevar(:, 7 + num_links_without_base : 9 + num_links_without_base);
    base_com_ang_vel = statevar(:, 10 + num_links_without_base : 12 + num_links_without_base);
    temp = base_com_ang_vel(:, 1);
    base_com_ang_vel(:, 1) = base_com_ang_vel(:, 2);
    base_com_ang_vel(:, 2) = base_com_ang_vel(:, 3);
    base_com_ang_vel(:, 3) = temp;
    %joint_velocity = statevar(:, 13 + num_links : 12 + 2 * num_links);
    base_sensor_position = zeros(len_time, 3);
    base_sensor_velocity = zeros(len_time, 3);
    for curr_instant = 1 : len_time
        phi = base_com_orientation(curr_instant, 1);
        theta = base_com_orientation(curr_instant, 2);
        psi = base_com_orientation(curr_instant, 3);
        rot_mat_base = eul2rotm([phi, theta, psi], 'ZYX');
        base_sensor_base_com_position = rot_mat_base * ...
            base_sensor_position_com_frame;
        base_sensor_position(curr_instant, :) = ...
        (base_com_position(curr_instant, 1:3).' + ...
            base_sensor_base_com_position).' - ...
            base_sensor_position_com_frame.';
        base_sensor_velocity(curr_instant, :) = ...
        (base_com_lin_vel(curr_instant, 1:3).' + ...
            cross(base_com_ang_vel(curr_instant, 1 : 3).', ...
            base_sensor_base_com_position)).';
    end
    
    % Save computed sensor position and velocity into statevar
    statevar(:, 1:3) = base_sensor_position;
    statevar(:, 7 + num_links_without_base : 9 + num_links_without_base)...
        = base_sensor_velocity;
    
    % Add noise to kinematic data
    add_noise = 0;
    base_sensor_position_std_dev = 1;
    base_sensor_orientation_std_dev = 1;
    joint_postion_std_dev = 1;
    base_lin_vel_std_dev = 1;
    base_ang_vel_std_dev = 1;
    joint_velocity_std_dev = 1;
    std_dev = [base_sensor_position_std_dev*ones(1,3), ...
        base_sensor_orientation_std_dev*ones(1,3), ...
        joint_postion_std_dev*ones(1, num_links_without_base), ...
        base_lin_vel_std_dev*ones(1,3), ...
        base_ang_vel_std_dev*ones(1,3), ...
        joint_velocity_std_dev*ones(1, num_links_without_base)];
    num_states = 12 + 2 * num_links_without_base;
    noise(len_time, num_states) = 0;
    for curr_state = 1 : num_states
        noise(:, curr_state) = ...
            add_noise*std_dev(curr_state)*randn(len_time, 1);
    end
    statevar = statevar(:, 1:num_states) + noise;
    save(strcat(curr_dir, robot_make, '/sim_real_data', '/statevar.dat'),...
        'statevar', '-ascii');
    
    % Results verification
    plot(statevar(:, 1), statevar(:, 2), 'r.', 'LineWidth', 3);
    hold on;
    quiver(statevar(:, 1), statevar(:, 2), ...
        statevar(:, 7 + num_links_without_base), statevar(:, 8 + num_links_without_base), ...
        'g--');
    legend('Position', 'Velocity');
    title('Sensor kinematic data in the sensor-based inertial frame');
    grid on;
end

