function [reg_mat] = regressor_matrix(robot_make,...
                     base_sensor_base_frame_position_base_frame, ...
                     statevar, mtvar)
    
    % Load robot structural and dynamic parameter data
    curr_dir = pwd;
    cd(strcat('../test_case_data', robot_make, '/config_files'));
    [num_links, not_planar, joint_twist_angle, link_length_DH, ...
        joint_offset, parent_link_index, link_x_com, link_y_com, link_z_com, ...
        ~, ~, link_mass, ~, inertia_xx_com, inertia_yy_com, inertia_zz_com, ...
        inertia_xy_com, inertia_yz_com, inertia_zx_com] = inputs();
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
    num_instants = length(statevar);
    
    % Compute orientation of link frames - Rotation matrix
    rot_mat_link_prev_link_frame = zeros(3, 3, num_instants, num_links);
    for curr_instant = 1 : num_instants
        curr_link_index = 1;
        phi = base_sensor_orientation(1, curr_instant);
        theta = base_sensor_orientation(2, curr_instant);
        psi = base_sensor_orientation(3, curr_instant);
        rot_mat_link_prev_link_frame(:, :, curr_instant, curr_link_index) = ...
            ZXY_euler_to_rot_mat(phi, theta, psi);
        for curr_link_index = 2 : num_links
            curr_joint_index = curr_link_index - 1;
            curr_joint_angle = joint_ang_position(curr_joint_index, curr_instant);
            curr_joint_twist = joint_twist_angle(curr_link_index);
            rot_mat_link_prev_link_frame(:, :, curr_instant, curr_link_index) = ...
                DH_to_rot_mat(curr_joint_twist, curr_joint_angle);
        end
    end
    
    rot_mat_link = zeros(3, 3, num_instants, num_links);
    rot_mat_link(:, :, :, 1) = rot_mat_link_prev_link_frame(:, :, :, 1);
    for curr_instant = 1 : num_instants
        for curr_link_index = 2 : num_links
            pli = parent_link_index(curr_link_index);
            rot_mat_prev_link = rot_mat_link(:, :, curr_instant, pli);
            rot_mat_link(:, :, curr_instant, curr_link_index) = ...
                rot_mat_prev_link * ...
                rot_mat_link_prev_link_frame(:, :, curr_instant, curr_link_index);
        end
    end
                
    % Compute link frame position vector
    link_frame_position = zeros(3, num_instants, num_links);
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
    link_frame_lin_velocity = zeros(3, num_instants, num_links);
    link_frame_ang_velocity = zeros(3, num_instants, num_links);
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
    
    param_vector = zeros(10 * num_links, 1);
    link_x_com(1) = -base_sensor_base_frame_position_base_frame(1);
    link_y_com(1) = -base_sensor_base_frame_position_base_frame(2);
    link_z_com(1) = -base_sensor_base_frame_position_base_frame(3);
    % param_vector_link = [Iaxx, Iayy, Iazz, Iaxy, Iayz, Iazx, m, max, may, maz].'
    for curr_link_index = 1 : num_links
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
    
    % Regressor matrix assembly
    reg_mat = zeros(6 * num_instants, 10 * num_links);
    for curr_instant = 1 : num_instants
        curr_instant_reg_mat = zeros(6, 10 * num_links);
        for curr_link_index = 1 : num_links
            submatrix_11 = zeros(3, 6);
            submatrix_12 = link_frame_lin_velocity(:, curr_instant, curr_link_index);
            ang_vel_matrix = vec_to_mat(link_frame_ang_velocity(:, curr_instant, curr_link_index));
            submatrix_13 = ang_vel_matrix * rot_mat_link(:, :, curr_instant, curr_link_index);
            submatrix_21 = rot_mat_link(:, :, curr_instant, curr_link_index) * ...
                bullet(rot_mat_link(:, :, curr_instant, curr_link_index).' * ...
                link_frame_ang_velocity(:, curr_instant, curr_link_index));
            submatrix_22 = vec_to_mat(link_frame_position(:, curr_instant, curr_link_index)) * ...
                link_frame_lin_velocity(:, curr_instant, curr_link_index);
            submatrix_23 = vec_to_mat(vec_to_mat(link_frame_position(:, curr_instant, curr_link_index)) * ...
                link_frame_ang_velocity(:, curr_instant, curr_link_index) - submatrix_12) * ...
                rot_mat_link(:, :, curr_instant, curr_link_index);
            link_mat = [submatrix_11, submatrix_12, submatrix_13; ...
                        submatrix_21, submatrix_22, submatrix_23];
            curr_instant_reg_mat(:, 10 * curr_link_index - 9 : 10 * curr_link_index) = ...
                link_mat;
        end
        reg_mat(6 * curr_instant - 5 : 6 * curr_instant, :) = curr_instant_reg_mat;
    end
     
    % Kinematic data verification    
    for curr_link_index = 1 : num_links
        figure(1);
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
    end
    axis('square'); grid on;
    
    % Momentum verification
    computed_mtum = reg_mat * param_vector;
    param_vector
        
%     if ~not_planar
%         num_dims = 3;
%         mtvar(:, [3, 4, 5]) = [];
%     else
%         num_dims = 6;
%     end
    
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
    
    % Delete Px, Py, Lz and zero comps
%     if ~not_planar
%         reg_mat(3 : 3 : 3 * num_instants,  :) = [];
%         reg_mat(:, 4 : 4 : 4 * num_links_with_base) = [];
%     end

end

function rot_mat = ZXY_euler_to_rot_mat(phi, theta, psi)
    rot_mat = [cos(phi) * cos(psi) - sin(phi) * sin(psi) * sin(theta)   -cos(theta) * sin(phi)   cos(phi) * sin(psi) + cos(psi) * sin(phi) * sin(theta)
               cos(psi) * sin(phi) + cos(phi) * sin(psi) * sin(theta)    cos(phi) * cos(theta)   sin(phi) * sin(psi) - cos(phi) * cos(psi) * sin(theta) 
              -cos(theta) * sin(psi)                                     sin(theta)              cos(psi) * cos(theta)                          ];
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
    I_new = I_old + m * (pos_vec.' * pos_vec * eye(3) - pos_vec * pos_vec.');
end

function I_vec = inertia_mat_to_vec(I)
    I_vec = [I(1, 1) I(2, 2) I(3, 3) I(1, 2) I(2, 3) I(3, 1)].'; 
end

function rot_mat = DH_to_rot_mat(joint_twist, joint_position)
    rot_mat = [cos(joint_position)                   -sin(joint_position)                     0; ...
               sin(joint_position) * cos(joint_twist) cos(joint_position) * cos(joint_twist) -sin(joint_twist); ... 
               sin(joint_position) * sin(joint_twist) cos(joint_position) * sin(joint_twist)  cos(joint_twist)];
end

function lin_vel = angular_to_linear_vel(ref_frame_position, ...
    link_frame_position, ref_joint_ang_velocity)
    lin_vel = cross(ref_joint_ang_velocity, link_frame_position - ref_frame_position);
end
        