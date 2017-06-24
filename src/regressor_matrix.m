function [reg_mat] = regressor_matrix(robot_make,...
                     base_sensor_position_base_frame, statevar, mtvar)
    
    % Load robot structural and dynamic parameter data
    curr_dir = pwd;
    cd(strcat('../test_case_data', robot_make, '/config_files'));
    [num_links_with_base, not_planar, ~, link_length_DH, ~, parent_link_index, ...
        link_x_com, link_y_com, link_z_com, link_length, ~, link_mass, ~, ...
        inertia_xx_com, inertia_yy_com, inertia_zz_com, ...
        inertia_xy_com, inertia_yz_com, inertia_zx_com] = inputs();
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
    reg_mat = zeros(6 * num_instants, 10 * num_links_with_base);
    for curr_instant = 1 : num_instants
        curr_instant_reg_mat = zeros(6, 10 * num_links_with_base);
        for curr_link_index = 1 : num_links_with_base
            submatrix_11 = zeros(3, 6);
            submatrix_12 = squeeze(joint_lin_velocity(:, :, curr_instant, curr_link_index));
            ang_vel_matrix = vec_to_mat(joint_ang_velocity(:, :, curr_instant, curr_link_index));
            submatrix_13 = ang_vel_matrix * rot_mat_link(:, :, curr_instant, curr_link_index);
            submatrix_21 = rot_mat_link(:, :, curr_instant, curr_link_index) * ...
                bullet(rot_mat_link(:, :, curr_instant, curr_link_index) * ...
                joint_ang_velocity(:, :, curr_instant, curr_link_index));
            submatrix_22 = vec_to_mat(joint_position(:, :, curr_instant, curr_link_index)) * ...
                joint_lin_velocity(:, :, curr_instant, curr_link_index);
            submatrix_23 = vec_to_mat(vec_to_mat(joint_position(:, :, curr_instant, curr_link_index)) * ...
                joint_ang_velocity(:, :, curr_instant, curr_link_index) - submatrix_12) * ...
                rot_mat_link(:, :, curr_instant, curr_link_index);
            link_mat = [submatrix_11, submatrix_12, submatrix_13; ...
                        submatrix_21, submatrix_22, submatrix_23];
            curr_instant_reg_mat(:, 10 * curr_link_index - 9 : 10 * curr_link_index) = ...
                link_mat;
        end
        reg_mat(6 * curr_instant - 5 : 6 * curr_instant, :) = curr_instant_reg_mat;
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
    axis('square'); grid on;
    
    % Momentum verification
    param_vector = zeros(10 * num_links_with_base, 1);
    link_x_com(1) = -base_sensor_position_base_frame(1);
    link_y_com(1) = -base_sensor_position_base_frame(2);
    link_z_com(1) = -base_sensor_position_base_frame(3);
    % param_vector_link = [Iaxx, Iayy, Iazz, Iaxy, Iayz, Iazx, m, max, may, maz].'
    for curr_link_index = 1 : num_links_with_base
        I_mat_com = [inertia_xx_com(curr_link_index) inertia_xy_com(curr_link_index) inertia_zx_com(curr_link_index); ...
                     inertia_xy_com(curr_link_index) inertia_yy_com(curr_link_index) inertia_yz_com(curr_link_index); ...
                     inertia_zx_com(curr_link_index) inertia_yz_com(curr_link_index) inertia_zz_com(curr_link_index)];
        link_com_link_frame_position = [link_x_com(curr_link_index),...
            link_y_com(curr_link_index), link_z_com(curr_link_index)].';
        I_mat_joint = move_inertia_axis(I_mat_com, link_mass(curr_link_index), link_com_link_frame_position);
        I_vec_joint = inertia_mat_to_vec(I_mat_joint);
        param_vector(10 * (curr_link_index - 1) + 1 : 10 * (curr_link_index), :) = ...
            [I_vec_joint; link_mass(curr_link_index); link_mass(curr_link_index) * link_com_link_frame_position];
    end
    computed_mtum = reg_mat * param_vector;
        
    if ~not_planar
        num_dims = 3;
%         mtvar(:, [3, 4, 5]) = [];
    else
        num_dims = 6;
    end
    
    for direction_index = 1 : 6
        directional_mtum_computed = ...
            computed_mtum(direction_index : 6 : 6 * num_instants);
        directional_mtum_actual = mtvar(:, direction_index);
        directional_mtum_error =  directional_mtum_actual - ...
            directional_mtum_computed;
        figure();
        plot(directional_mtum_computed); hold on;
        plot(directional_mtum_actual);
        plot(directional_mtum_error, 'r');
        legend('Computed momentum', 'Actual momentum', 'Error'); grid on;   
    end
    
    % Delete z-components
    if ~not_planar
        reg_mat(3 : 3 : 3 * num_instants,  :) = [];
        reg_mat(:, 4 : 4 : 4 * num_links_with_base) = [];
    end

end

function skew_symm_matrix = vec_to_mat(vector)
    skew_symm_matrix = [0          -vector(3)  vector(2); ...
                        vector(3)   0         -vector(1); ...
                       -vector(2)  vector(1)  0];
end

function bullet_notation = bullet(omg)
    bullet_notation = [omg(1) 0 0 omg(2) 0 omg(3); ...
                       0 omg(2) 0 omg(1) omg(3) 0; ...
                       0 0 omg(3) 0 omg(2) omg(1)];
end

function I_new = move_inertia_axis(I_old, m, pos_vec)
    I_new = I_old - m * vec_to_mat(pos_vec) ^ 2;
end

function I_vec = inertia_mat_to_vec(I)
    I_vec = [I(1, 1) I(2, 2) I(3, 3) I(1, 2) I(2, 3) I(3, 1)].'; 
end
