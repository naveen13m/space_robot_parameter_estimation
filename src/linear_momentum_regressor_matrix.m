function [reg_mat] = linear_momentum_regressor_matrix(robot_make,...
                     base_sensor_position_base_frame, statevar, mtvar)
    
    % Load robot structural and dynamic parameter data
    curr_dir = pwd;
    cd(strcat('../test_case_data', robot_make, '/config_files'));
    [num_links_with_base, ~, ~, link_length_DH, ~, parent_link_index, ...
        link_x_com, link_y_com, link_z_com, link_length, ~, link_mass] = ...
        inputs();
    cd(curr_dir);
    
    % Data assignment
    arm_initial_link_index = find(parent_link_index == 1) - 1;
    num_arms = 0;
    for curr_arm_initial_link_index = arm_initial_link_index
        num_arms = num_arms + 1;
        base_arm_joint_position_base_frame = ...
            [link_length_DH(curr_arm_initial_link_index + 1), 0, 0]';
        base_arm_joint_position_sensor_frame(:, num_arms) = ...
            base_arm_joint_position_base_frame ...
            - base_sensor_position_base_frame;
    end
    num_links_without_base = num_links_with_base - 1;
    base_sensor_position = statevar(:, 1 : 3);
    base_sensor_orientation = statevar(:, 4 : 6);
    rev_joint_ang_position = statevar(:, 7 : 6 + num_links_without_base);
    base_sensor_lin_velocity = statevar(:, 7 + num_links_without_base :...
        9 + num_links_without_base);
    base_sensor_ang_velocity = statevar(:, 10 + num_links_without_base :...
        12 + num_links_without_base);
    rev_joint_ang_velocity = statevar(:, 13 + num_links_without_base :...
        12 + 2 * num_links_without_base);
    num_instants = length(statevar);
    joint_ang_position_prev_joint_frame = [base_sensor_orientation(:,1), ...
        rev_joint_ang_position];
    joint_ang_velocity_prev_joint_frame = [base_sensor_ang_velocity(:, 1), ...
        rev_joint_ang_velocity];
    
    % Compute rotation matrix
    rot_mat_link_prev_link_frame = ...
        zeros(3, 3, num_instants, num_links_with_base);
    for curr_instant = 1 : num_instants
        for curr_link = 1 : num_links_with_base
            joint_angle = joint_ang_position_prev_joint_frame(curr_instant, curr_link);
            rot_mat_link_prev_link_frame(:, :, curr_instant,...
                curr_link) = [cos(joint_angle) -sin(joint_angle) 0
                              sin(joint_angle)  cos(joint_angle) 0
                              0                 0                1];
        end
    end
    
    rot_mat_link = zeros(3, 3, num_instants, num_links_with_base);
    rot_mat_link(:, :, :, 1) = rot_mat_link_prev_link_frame(:, :, :, 1);
    start_end_link_index = [arm_initial_link_index, num_links_with_base];
    for curr_instant = 1 : num_instants
        for curr_arm_index = 1 : num_arms
            for curr_joint_index = start_end_link_index(curr_arm_index) : ...
                    start_end_link_index(curr_arm_index + 1) - 1
                if curr_joint_index ==  start_end_link_index(curr_arm_index)
                    rot_mat_prev = rot_mat_link(:, :, curr_instant, 1);
                else
                    rot_mat_prev = rot_mat_link(:, :, curr_instant,...
                        curr_joint_index);
                end
                rot_mat_link(:, :, curr_instant, curr_joint_index + 1) = ...
                    rot_mat_prev * ...
                    rot_mat_link_prev_link_frame(:, :, curr_instant, curr_joint_index + 1);
            end
        end
    end
                
    % Compute joint position vector
    joint_position = zeros(3, 1, num_instants, num_links_with_base);
    joint_position(:, :, :, 1) = base_sensor_position.';
    base_arm_joint_base_sensor = zeros(3, 1, num_instants, curr_arm_index);
    
    for curr_instant = 1 : num_instants
        for curr_arm_index = 1 : num_arms
            base_arm_joint_base_sensor(:, :, curr_instant, curr_arm_index) ...
                = rot_mat_link(:, :, curr_instant, 1) * ...
                base_arm_joint_position_sensor_frame(:, curr_arm_index);
            for curr_joint_index = start_end_link_index(curr_arm_index) : ...
                    start_end_link_index(curr_arm_index + 1) - 1
                if curr_joint_index ==  start_end_link_index(curr_arm_index)
                    curr_joint_prev_joint_position = ...
                        base_arm_joint_base_sensor(:, :, curr_instant, curr_arm_index);
                    prev_joint_position = joint_position(:, :, curr_instant, 1);
                else
                    curr_joint_prev_joint_position = ...
                        rot_mat_link(:, :, curr_instant, curr_joint_index) * ...
                        ([link_length(curr_joint_index), 0, 0].');
                    prev_joint_position = ...
                        joint_position(:, :, curr_instant, curr_joint_index);
                end
                joint_position(:, :, curr_instant, curr_joint_index + 1) = ...
                    prev_joint_position + curr_joint_prev_joint_position;
            end
        end
    end
    
    % Compute joint linear and angular velocity in inertial frame
    joint_lin_velocity = zeros(3, 1, num_instants, num_links_with_base);
    joint_ang_velocity = zeros(3, 1, num_instants, num_links_with_base);
    joint_lin_velocity(:, :, :, 1) = base_sensor_lin_velocity.';
    joint_ang_velocity(:, :, :, 1) = [zeros(num_instants, 1), ...
        zeros(num_instants, 1), joint_ang_velocity_prev_joint_frame(:, 1)].';
    
    for curr_instant = 1 : num_instants
        for curr_arm_index = 1 : num_arms
            for curr_joint_index = start_end_link_index(curr_arm_index) : ...
                    start_end_link_index(curr_arm_index + 1) - 1
                base_angular_velocity = ...
                    [0; 0; joint_ang_velocity_prev_joint_frame(curr_instant, 1)];
                joint_lin_velocity(:, :, curr_instant, curr_joint_index + 1) = ...
                    joint_lin_velocity(:, :, curr_instant, 1) + ...
                    cross(base_angular_velocity, ...
                    (joint_position(:, :, curr_instant, curr_joint_index + 1) - ...
                    joint_position(:, :, curr_instant, 1)));
                joint_ang_velocity(:, :, curr_instant, curr_joint_index + 1) = ...
                    base_angular_velocity;
                for iter_joint_index = start_end_link_index(curr_arm_index) : ...
                        curr_joint_index
                    joint_angular_velocity_prev_joint_frame = ...
                    [0; 0; joint_ang_velocity_prev_joint_frame(curr_instant, iter_joint_index + 1)];
                    joint_lin_velocity(:, :, curr_instant, curr_joint_index + 1) = ...
                        joint_lin_velocity(:, :, curr_instant, curr_joint_index + 1) + ...
                        cross(joint_angular_velocity_prev_joint_frame, ...
                        (joint_position(:, :, curr_instant, curr_joint_index + 1) - ...
                        joint_position(:, :, curr_instant, iter_joint_index + 1)));
                    joint_ang_velocity(:, :, curr_instant, curr_joint_index + 1) = ...
                        joint_ang_velocity(:, :, curr_instant, curr_joint_index + 1) + ...
                        [0; 0; joint_ang_velocity_prev_joint_frame(curr_instant, iter_joint_index + 1)];
                end
            end
        end
    end
    
    % Regressor matrix assembly
    reg_mat = [];
    for curr_instant = 1 : num_instants
        curr_instant_reg_mat = [];
        for curr_link_index = 1 : num_links_with_base
            lin_vel_vector = squeeze(joint_lin_velocity(:, :, curr_instant, curr_link_index));
            ang_vel_matrix = vec_to_mat(joint_ang_velocity(:, :, curr_instant, curr_link_index));
            ang_vel_reg_matrix = ang_vel_matrix * rot_mat_link(:, :, curr_instant, curr_link_index);
            link_mat = [lin_vel_vector, ang_vel_reg_matrix];
            curr_instant_reg_mat = [curr_instant_reg_mat, link_mat];
        end
        reg_mat = [reg_mat; ...
                   curr_instant_reg_mat];
    end
    
    
    % Kinematic data verification    
    for curr_joint_index = 1 : num_links_with_base
        figure(1);
        joint_pos_x_coord = squeeze(joint_position(1, 1, :, curr_joint_index));
        joint_pos_y_coord  = squeeze(joint_position(2, 1, :, curr_joint_index));
        joint_lin_vel_x_comp = squeeze(joint_lin_velocity(1, 1, :, curr_joint_index));
        joint_lin_vel_y_comp = squeeze(joint_lin_velocity(2, 1, :, curr_joint_index));
        plot(joint_pos_x_coord , joint_pos_y_coord, 'r.', 'LineWidth', 5);
        hold on;
        quiver(joint_pos_x_coord , joint_pos_y_coord, joint_lin_vel_x_comp, ... 
            joint_lin_vel_y_comp, 'g--');
    end
    figure(1);
    axis('square'); grid on;
    
    % Momentum verification
    param_vector = zeros(4 * num_links_with_base, 1);
    link_mass_index = 1 : 4 : 4 * num_links_with_base;
    link_x_com_index = 2 : 4 : 4 * num_links_with_base;
    link_y_com_index = 3 : 4 : 4 * num_links_with_base;
    link_z_com_index = 4 : 4 : 4 * num_links_with_base;
    param_vector(link_mass_index) = link_mass;
    param_vector(2:4) = -link_mass(1) * base_sensor_position_base_frame;
    for curr_link_index = 2 : num_links_with_base
        param_vector([link_x_com_index(curr_link_index), ...
            link_y_com_index(curr_link_index), ...
            link_z_com_index(curr_link_index)], :) = link_mass(curr_link_index) * ...
            [link_x_com((curr_link_index)), link_y_com((curr_link_index)), ...
            link_z_com((curr_link_index))].';
    end
    computed_lin_mtum = reg_mat * param_vector;
    x_lin_mtum_computed = computed_lin_mtum(1 : 3 : 3 * num_instants);
    y_lin_mtum_computed = computed_lin_mtum(1 : 3 : 3 * num_instants);
%     z_lin_mtum_computed = computed_lin_mtum(1 : 3 : 3 * num_instants);
    x_lin_mtum_actual = mtvar(:, 1);
    y_lin_mtum_actual = mtvar(:, 2);
    x_lin_mtum_error = x_lin_mtum_actual - x_lin_mtum_computed;
    y_lin_mtum_error = y_lin_mtum_actual - y_lin_mtum_computed;
    figure(2);
    plot(x_lin_mtum_computed); hold on;
    plot(y_lin_mtum_computed);
    legend('Computed momentum', 'Actual momentum'); grid on;
    figure(3);
    plot(x_lin_mtum_actual); hold on;
    plot(y_lin_mtum_actual);
    legend('Computed momentum', 'Actual momentum'); grid on;
    figure(4);
    plot(x_lin_mtum_error, 'r');
    hold on;
    plot(y_lin_mtum_error, 'b');
%     plot(z_lin_mtum_computed, 'g');
    legend('x','y'); grid on; 

end

function skew_symm_matrix = vec_to_mat(vector)
    skew_symm_matrix = [0          -vector(3)  vector(2); ...
                        vector(3)   0         -vector(1); ...
                       -vector(2)  vector(1)  0];
end