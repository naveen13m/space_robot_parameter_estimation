function global_kin_mat = global_kinematic_matrix(robot_make,...
                     base_sensor_base_frame_position_base_frame, ...
                     statevar, mtvar)
    
    % Load robot structural and dynamic parameter data
    curr_dir = pwd;
    cd(strcat('../test_case_data', robot_make, '/config_files'));
    [num_links, not_planar, joint_twist_angle, link_length_DH, ...
        joint_offset, parent_link_index, link_x_com, link_y_com, link_z_com, ...
        ~, ~, link_mass, ~, inertia_xx_com, inertia_yy_com, inertia_zz_com, ...
        inertia_xy_com, inertia_yz_com, inertia_zx_com] = inputs();
    link_x_com(1) = -base_sensor_base_frame_position_base_frame(1);
    link_y_com(1) = -base_sensor_base_frame_position_base_frame(2);
    link_z_com(1) = -base_sensor_base_frame_position_base_frame(3);
    cd(curr_dir);
    
    % Data assignment
    arm_initial_link_index = find(parent_link_index == 1);
    arm_terminal_link_index = [arm_initial_link_index(2 : end) - 1, num_links];
    num_arms = length(arm_initial_link_index);
    base_arm_joint_sensor_frame_position_sensor_frame = zeros(3, num_arms);
    curr_arm_index = 0;
    for curr_arm_initial_link_index = arm_initial_link_index
        curr_arm_index = curr_arm_index + 1;
        base_arm_joint_sensor_frame_position_sensor_frame(:,  curr_arm_index) = ...
            [link_length_DH(curr_arm_initial_link_index);
            -joint_offset(curr_arm_initial_link_index) * ...
            sin(joint_twist_angle(curr_arm_initial_link_index)); ...
             joint_offset(curr_arm_initial_link_index) * ...
             cos(joint_twist_angle(curr_arm_initial_link_index))];
        base_arm_joint_sensor_frame_position_sensor_frame(:,  curr_arm_index) = ...
            base_arm_joint_sensor_frame_position_sensor_frame(:,  curr_arm_index) ...
            - base_sensor_base_frame_position_base_frame;
    end
    num_links_without_base = num_links - 1;
    base_sensor_position = statevar(:, 1 : 3).';
    base_sensor_orientation = statevar(:, 4 : 6).'; % ZXY order
    joint_ang_position = statevar(:, 7 : 6 + num_links_without_base).';
    base_sensor_lin_velocity = statevar(:, 7 + num_links_without_base :...
        9 + num_links_without_base).'; 
    base_sensor_ang_velocity = statevar(:, 10 + num_links_without_base :...
        12 + num_links_without_base).'; 
    joint_ang_speed = statevar(:, 13 + num_links_without_base :...
        12 + 2 * num_links_without_base).';
    num_instants = size(statevar, 1);
    
    % Regressor matrix data initialization
    rot_mat_link_prev_link_frame = zeros(3, 3, num_instants, num_links);
    rot_mat_link = zeros(3, 3, num_instants, num_links);
    link_frame_position = zeros(3, num_instants, num_links);
    link_frame_lin_velocity = zeros(3, num_instants, num_links);
    link_frame_ang_velocity = zeros(3, num_instants, num_links);
    
    % Compute orientation of link frames - Rotation matrix
    for curr_instant = 1 : num_instants
        curr_link_index = 1;
        phi = base_sensor_orientation(1, curr_instant);
        theta = base_sensor_orientation(2, curr_instant);
        psi = base_sensor_orientation(3, curr_instant);
        rot_mat_link_prev_link_frame(:, :, curr_instant, curr_link_index) = ...
            ZXY_euler_to_rot_mat(phi, theta, psi);
        rot_mat_link(:, :, curr_instant, 1) = ...
            rot_mat_link_prev_link_frame(:, :, curr_instant, 1);
        for curr_link_index = 2 : num_links
            curr_joint_index = curr_link_index - 1;
            curr_joint_angle = joint_ang_position(curr_joint_index, curr_instant);
            curr_joint_twist = joint_twist_angle(curr_link_index);
            rot_mat_link_prev_link_frame(:, :, curr_instant, curr_link_index) = ...
                DH_to_rot_mat(curr_joint_twist, curr_joint_angle);
            pli = parent_link_index(curr_link_index);
            rot_mat_prev_link = rot_mat_link(:, :, curr_instant, pli);
            rot_mat_link(:, :, curr_instant, curr_link_index) = ...
                rot_mat_prev_link * ...
                rot_mat_link_prev_link_frame(:, :, curr_instant, curr_link_index);
        end
    end
    
    % Compute link frame position vector
    link_frame_position(:, :, 1) = base_sensor_position;
    base_arm_joint_base_sensor_position = zeros(3, num_instants, num_arms);
    for curr_instant = 1 : num_instants
        for curr_arm_index = 1 : num_arms
            base_arm_joint_base_sensor_position(:, curr_instant, curr_arm_index) ...
                = rot_mat_link(:, :, curr_instant, 1) * ...
                base_arm_joint_sensor_frame_position_sensor_frame(:, curr_arm_index);            
            for curr_link_index = arm_initial_link_index(curr_arm_index) : ...
                    arm_terminal_link_index(curr_arm_index)
                if curr_link_index ==  arm_initial_link_index(curr_arm_index)
                    curr_link_frame_prev_link_frame_position = ...
                        base_arm_joint_base_sensor_position(:, curr_instant, curr_arm_index);
                    prev_link_frame_position = link_frame_position(:, curr_instant, 1);
                else
                    curr_link_frame_prev_link_frame_position_prev_link_frame = ...
                        [link_length_DH(curr_link_index);
                         -joint_offset(curr_link_index) * ...
                          sin(joint_twist_angle(curr_link_index)); ...
                          joint_offset(curr_link_index) * ...
                          cos(joint_twist_angle(curr_link_index))];
                    curr_link_frame_prev_link_frame_position = ...
                        rot_mat_link(:, :, curr_instant, curr_link_index - 1) * ...
                        curr_link_frame_prev_link_frame_position_prev_link_frame; 
                    prev_link_frame_position = ...
                        link_frame_position(:, curr_instant, curr_link_index - 1);
                end
                link_frame_position(:, curr_instant, curr_link_index) = ...
                    prev_link_frame_position + curr_link_frame_prev_link_frame_position;
            end
        end
    end
    
    % Compute link frame linear and angular velocity in inertial frame
    link_frame_lin_velocity(:, :, 1) = base_sensor_lin_velocity;
    link_frame_ang_velocity(:, :, 1) = base_sensor_ang_velocity;
    for curr_instant = 1 : num_instants
        for curr_arm_index = 1 : num_arms
            for curr_link_index = arm_initial_link_index(curr_arm_index) : ...
                    arm_terminal_link_index(curr_arm_index)
                link_frame_lin_velocity(:, curr_instant, curr_link_index) = ...
                    link_frame_lin_velocity(:, curr_instant, 1) + ...
                    angular_to_linear_vel(link_frame_position(:, curr_instant, 1), ...
                    link_frame_position(:, curr_instant, curr_link_index),...
                    base_sensor_ang_velocity(:, curr_instant));
                link_frame_ang_velocity(:, curr_instant, curr_link_index) = ...
                    base_sensor_ang_velocity(:, curr_instant);
                for iter_link_index = arm_initial_link_index(curr_arm_index) : ...
                        curr_link_index
                    iter_joint_index = iter_link_index - 1;
                    iter_link_angular_velocity = ...
                        rot_mat_link(:, :, curr_instant, iter_link_index) * ...
                        [0; 0; joint_ang_speed(iter_joint_index, curr_instant)];
                    link_frame_lin_velocity(:, curr_instant, curr_link_index) = ...
                        link_frame_lin_velocity(:, curr_instant, curr_link_index) + ...
                        angular_to_linear_vel(link_frame_position(:, curr_instant, iter_link_index), ...
                        link_frame_position(:, curr_instant, curr_link_index),...
                        iter_link_angular_velocity);
                    link_frame_ang_velocity(:, curr_instant, curr_link_index) = ...
                        link_frame_ang_velocity(:, curr_instant, curr_link_index) + ...
                        iter_link_angular_velocity;
                end
            end
        end
    end
    
    % Regressor matrix assembly
    num_equs = 6;
%     num_equs = 9; %If CoM equs are included
    num_link_params = 10;
    global_kin_mat = zeros(num_equs * num_instants, num_link_params * num_links);
    for curr_instant = 1 : num_instants
        for curr_link_index = 1 : num_links
            submatrix_11 = zeros(3, 6);
            submatrix_12 = link_frame_lin_velocity(:, curr_instant, curr_link_index);
            ang_vel_matrix = vec_to_mat(link_frame_ang_velocity(:, curr_instant, curr_link_index));
            submatrix_13 = ang_vel_matrix * rot_mat_link(:, :, curr_instant, curr_link_index);
            submatrix_21 = rot_mat_link(:, :, curr_instant, curr_link_index) * ...
                bullet(rot_mat_link(:, :, curr_instant, curr_link_index).' * ...
                link_frame_ang_velocity(:, curr_instant, curr_link_index));
            submatrix_22 = cross(link_frame_position(:, curr_instant, curr_link_index), submatrix_12);
            submatrix_23 = (vec_to_mat(link_frame_position(:, curr_instant, curr_link_index)) * ...
                vec_to_mat(link_frame_ang_velocity(:, curr_instant, curr_link_index)) - ...
                vec_to_mat(submatrix_12)) * rot_mat_link(:, :, curr_instant, curr_link_index);
            link_kin_mat = [submatrix_11, submatrix_12, submatrix_13; ...
                            submatrix_21, submatrix_22, submatrix_23];
%                         submatrix_31, submatrix_32, submatrix_33];
            system_kin_mat(:, num_link_params * (curr_link_index - 1) + 1 :...
                num_link_params * curr_link_index) = link_kin_mat;
        end
        global_kin_mat(num_equs * (curr_instant - 1) + 1 : num_equs * curr_instant, :) = system_kin_mat;
    end
     
    % Kinematic data and momentum verification    
    plot_kinematic_data(num_links, link_frame_position, link_frame_lin_velocity,...
        link_mass, link_x_com, link_y_com, link_z_com, rot_mat_link, num_instants);
    param_vector = construct_dyn_param_vector(...
        inertia_xx_com, inertia_yy_com, inertia_zz_com, inertia_xy_com, inertia_yz_com,...
        inertia_zx_com, link_mass, link_x_com, link_y_com, link_z_com, num_links);
    computed_mtum = global_kin_mat * param_vector;
    plot_momentum_data(num_instants, num_equs, computed_mtum, mtvar, 6);
end

function plot_kinematic_data(num_links, link_frame_position, link_frame_lin_velocity,...
        link_mass, link_x_com, link_y_com, link_z_com, rot_mat_link, num_instants)
    figure();
    sys_pos_x_coord = zeros(1, size(link_frame_position, 2));
    sys_pos_y_coord = zeros(1, size(link_frame_position, 2));
    sys_pos_z_coord = zeros(1, size(link_frame_position, 2));
    link_com_link_frame_position = zeros(3, num_instants, num_links);
    for curr_link_index = 1 : num_links
        link_pos_x_coord = link_frame_position(1, :, curr_link_index);
        link_pos_y_coord  = link_frame_position(2, :, curr_link_index);
        link_pos_z_coord  = link_frame_position(3, :, curr_link_index);
        link_lin_vel_x_comp = link_frame_lin_velocity(1, :, curr_link_index);
        link_lin_vel_y_comp = link_frame_lin_velocity(2, :, curr_link_index);
        link_lin_vel_z_comp = link_frame_lin_velocity(3, :, curr_link_index);
        plot3(link_pos_x_coord , link_pos_y_coord, link_pos_z_coord, ...
            'r.', 'LineWidth', 5);
        hold on;
        quiver3(link_pos_x_coord , link_pos_y_coord, link_pos_z_coord, ...
            link_lin_vel_x_comp, link_lin_vel_y_comp, ...
            link_lin_vel_z_comp, 'g--');
        link_com_link_frame_position_link_frame = ...
                                        [link_x_com(curr_link_index);...
                                        link_y_com(curr_link_index);...
                                        link_z_com(curr_link_index)];
        for curr_instant = 1 : num_instants
            link_com_link_frame_position(:, curr_instant, curr_link_index) = ...
                rot_mat_link(:, :, curr_instant, curr_link_index) * ...
                link_com_link_frame_position_link_frame;
        end
        sys_pos_x_coord = sys_pos_x_coord + ...
            link_mass(curr_link_index) * (link_pos_x_coord + ...
            link_com_link_frame_position(1, :, curr_link_index));
        sys_pos_y_coord = sys_pos_y_coord + ...
            link_mass(curr_link_index) * (link_pos_y_coord + ...
            link_com_link_frame_position(2, :, curr_link_index));
        sys_pos_z_coord = sys_pos_z_coord + ...
            link_mass(curr_link_index) * (link_pos_z_coord + ...
            link_com_link_frame_position(3, :, curr_link_index));
    end
    sys_mass = sum(link_mass);
    sys_pos_x_coord = sys_pos_x_coord / sys_mass;
    sys_pos_y_coord = sys_pos_y_coord / sys_mass;
    sys_pos_z_coord = sys_pos_z_coord / sys_mass;
    plot3(sys_pos_x_coord , sys_pos_y_coord, sys_pos_z_coord, ...
            'k.', 'LineWidth', 2);
    set(gcf,'units','pixels','position',[0, 0, 1366, 768]);
    axis('square'); grid on; drawnow;
end

function plot_momentum_data(num_instants, num_equs, computed_mtum, mtvar, num_dims)
    figure();
    for direction_index = 1 : num_dims
        directional_mtum_computed = ...
            computed_mtum(direction_index : num_equs : num_equs * num_instants);
        directional_mtum_actual = mtvar(:, direction_index);
        directional_mtum_error =  directional_mtum_actual - ...
            directional_mtum_computed;
        subplot(2, 3, direction_index);
        plot(directional_mtum_computed); hold on;
        plot(directional_mtum_actual);
        plot(directional_mtum_error, 'r');
        grid on;
    end
    hL = legend('Computed momentum', 'Actual momentum', 'Error');
    set(hL,'Position', [0.50 0.48 0.05 0.05], 'Units', 'normalized');
    set(gcf,'units','pixels','position',[0, 0, 1366, 768]);
    drawnow;
end