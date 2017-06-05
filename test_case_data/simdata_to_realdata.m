function [statevar] = simdata_to_realdata(robot_make, ...
                                      base_link_sensor_position_base_frame)
    % Load simulation data
    current_dir = pwd;
    data_filename = {'/statevar.dat', '/timevar.dat', '/mtvar.dat'};
    num_files = length(data_filename);
    for current_instant = 1 : num_files
        try
            load(strcat(current_dir, robot_make, '/sim_data', ...
                data_filename{current_instant}));
        catch
            error(strcat(current_dir, robot_make, '/sim_data', ...
                data_filename{current_instant}, ' is not present'));
        end
    end
    
    % Compute base sensor frame data based on the base simulation frame data
    len_time = length(timevar);
    cd(strcat(current_dir, robot_make, '/config_files'));
    num_links = inputs();
    num_links = num_links - 1; % Excluding the base link
    cd(current_dir);
    base_link_position = statevar(:, 1 : 3);
    base_link_orientation = statevar(:, 4 : 6);
    %joint_position = statvar(:, 7 : 6 + num_link);
    base_link_lin_vel = statevar(:, 7 + num_links : 9 + num_links);
    base_link_ang_vel = statevar(:, 10 + num_links : 12 + num_links);
    temp = base_link_ang_vel(:, 1);
    base_link_ang_vel(:, 1) = base_link_ang_vel(:, 2);
    base_link_ang_vel(:, 2) = base_link_ang_vel(:, 3);
    base_link_ang_vel(:, 3) = temp;
    %joint_velocity = statevar(:, 13 + num_links : 12 + 2 * num_links);
    base_link_sensor_position = zeros(len_time, 3);
    base_link_sensor_velocity = zeros(len_time, 3);
    for current_instant = 1:len_time
        phi = base_link_orientation(current_instant);
        rot_mat_base = [cos(phi) -sin(phi) 0
                        sin(phi)  cos(phi) 0
                        0         0        1];
        base_link_sensor_position(current_instant, :) = ...
        (base_link_position(current_instant, 1:3).' + ...
            rot_mat_base*base_link_sensor_position_base_frame).';
        base_link_sensor_velocity(current_instant, :) = ...
        (base_link_lin_vel(current_instant, 1:3).' + ...
            cross(base_link_ang_vel(current_instant, 1 : 3).', ...
            rot_mat_base * base_link_sensor_position_base_frame)).';         
    end
    
    % Add noise to kinematic data
    base_link_position_std_dev = 1;
    base_link_orientation_std_dev = 1;
    joint_postion_std_dev = 1;
    base_link_lin_vel_std_dev = 1;
    base_link_ang_vel_std_dev = 1;
    joint_velocity_std_dev = 1;
    std_dev = [base_link_position_std_dev*ones(1,3), ...
        base_link_orientation_std_dev*ones(1,3), ...
        joint_postion_std_dev*ones(1, num_links), ...
        base_link_lin_vel_std_dev*ones(1,3), ...
        base_link_ang_vel_std_dev*ones(1,3), ...
        joint_velocity_std_dev*ones(1, num_links)];
    num_states = 12 + 2 * num_links;
    noise(len_time, num_states) = 0;
    for current_state = 1 : num_states
        noise(:, current_state) = std_dev(current_state)*randn(len_time, 1);
    end
    statevar = statevar(:, 1:num_states) + noise;
    save(strcat(current_dir, robot_make, '/real_data', '/statevar.dat'),...
        'statevar', '-ascii');
end

