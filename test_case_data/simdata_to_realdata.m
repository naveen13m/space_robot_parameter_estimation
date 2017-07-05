function simdata_to_realdata(robot_make, ...
                            base_sensor_base_com_position_base_com)
    % Load simulation data
    close all; clc;
    curr_dir = pwd;
    data_filename = {'/statevar.dat', '/timevar.dat', '/mtvar.dat'};
    num_data_files = length(data_filename);
    for curr_instant = 1 : num_data_files
        load(strcat(curr_dir, robot_make, '/sim_data', data_filename{curr_instant}));
    end
    
    % Compute base sensor data in the modified inertial frame
    num_instants = length(timevar);
    cd(strcat(curr_dir, robot_make, '/config_files'));
    num_links_with_base = inputs();
    num_links_without_base = num_links_with_base - 1;
    cd(curr_dir);
    base_com_position = statevar(:, 1 : 3);
    base_com_orientation = statevar(:, 4 : 6);
    base_com_lin_vel = statevar(:, 7 + num_links_without_base : 9 + num_links_without_base);
    base_com_eul_rates = statevar(:, 10 + num_links_without_base : 12 + num_links_without_base);
    base_sensor_position = zeros(num_instants, 3);
    base_sensor_lin_velocity = zeros(num_instants, 3);
    base_com_ang_vel = zeros(num_instants, 3);
    for curr_instant = 1 : num_instants
        phi = base_com_orientation(curr_instant, 1);
        theta = base_com_orientation(curr_instant, 2);
        psi = base_com_orientation(curr_instant, 3);
        rot_mat_base = ZXY_euler_to_rot_mat(phi, theta, psi);
        ZXY_euler_rates_to_body_rates = [-sin(psi)*cos(theta)   cos(psi)  0
                                          sin(theta)            0         1
                                          cos(psi)*cos(theta)   sin(psi)  0];
        base_com_ang_vel(curr_instant, :) = rot_mat_base * ...
            ZXY_euler_rates_to_body_rates * base_com_eul_rates(curr_instant, :).';
        base_sensor_base_com_position = rot_mat_base * ...
            base_sensor_base_com_position_base_com;
        base_sensor_position(curr_instant, :) = ...
        (base_com_position(curr_instant, 1:3).' + ...
            base_sensor_base_com_position).' - ...
            base_sensor_base_com_position_base_com.';
        base_sensor_lin_velocity(curr_instant, :) = ...
        (base_com_lin_vel(curr_instant, :).' + ...
            cross(base_com_ang_vel(curr_instant, :).', ...
            base_sensor_base_com_position)).';
    end
    
    % Save computed sensor position and velocity into statevar
    statevar(:, 1:3) = base_sensor_position;
    statevar(:, 7 + num_links_without_base : 9 + num_links_without_base)...
        = base_sensor_lin_velocity;
    statevar(:, 10 + num_links_without_base : 12 + num_links_without_base)...
        = base_com_ang_vel;
    
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
    noise(num_instants, num_states) = 0;
    for curr_state = 1 : num_states
        noise(:, curr_state) = ...
            add_noise*std_dev(curr_state)*randn(num_instants, 1);
    end
    statevar = statevar(:, 1:num_states) + noise;
    save(strcat(curr_dir, robot_make, '/sim_real_data', '/statevar.dat'),...
        'statevar', '-ascii');
    
    mtvar_modified = zeros(num_instants, 6);
    % Compute momentum in the modified inertia frame
    for curr_instant = 1 : num_instants
        old_mtum = mtvar(curr_instant, :);
        pos_vec = base_sensor_base_com_position_base_com.';
        mtvar_modified(curr_instant, :) = move_mtum_ref_frame(old_mtum, pos_vec);
    end
    save(strcat(curr_dir, robot_make, '/sim_real_data', '/mtvar.dat'),...
        'mtvar_modified', '-ascii');
    
    % Results verification
    plot3(statevar(:, 1), statevar(:, 2), statevar(:, 3), 'r.', 'LineWidth', 3);
    hold on;
    quiver3(statevar(:, 1), statevar(:, 2), statevar(:, 3), ...
        statevar(:, 7 + num_links_without_base), statevar(:, 8 + num_links_without_base), ...
        statevar(:, 9 + num_links_without_base), 'g--');
    legend('Position', 'Velocity');
    title('Sensor kinematic data in the sensor-based inertial frame');
    grid on;
end

function rot_mat = ZXY_euler_to_rot_mat(phi, theta, psi)
    rot_mat = [cos(phi)*cos(psi)-sin(phi)*sin(theta)*sin(psi)   -sin(phi)*cos(theta)     cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi)
               sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi)    cos(phi)*cos(theta)     sin(phi)*sin(psi)-cos(phi)*sin(theta)*cos(psi)
               -cos(theta)*sin(psi)                              sin(theta)              cos(theta)*cos(psi)                         ];
end

function new_mtum = move_mtum_ref_frame(old_mtum, pos_vec)
    new_mtum(:, 1 : 3) = old_mtum(:, 1 : 3);
    new_mtum(:, 4 : 6) = old_mtum(:, 4 : 6) - cross(pos_vec, new_mtum(:, 1 : 3));
end