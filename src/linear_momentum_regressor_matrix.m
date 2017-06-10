function [reg_mat] = linear_momentum_regressor_matrix(robot_make,...
    base_sensor_position_base_frame, statevar)
    
    % Load robot structural data
    curr_dir = pwd;
    cd(strcat('../test_case_data', robot_make, '/config_files'));
    [num_links_with_base, ~, ~, link_length_DH, ~, parent_link_index, ~, ~, ~, ... 
        link_length, ~] = inputs();
    cd(curr_dir);
    
    % Data assignment
    arm_initial_link_index = find(parent_link_index == 1) - 1;
    num_arms = 0;
    base_arm_joint_position_sensor_frame = zeros(3, 2);
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
    base_com_lin_vel = statevar(:, 7 + num_links_without_base :...
        9 + num_links_without_base);
    base_com_ang_vel = statevar(:, 10 + num_links_without_base :...
        12 + num_links_without_base);
    temp = base_com_ang_vel(:, 1);
    base_com_ang_vel(:, 1) = base_com_ang_vel(:, 2);
    base_com_ang_vel(:, 2) = base_com_ang_vel(:, 3);
    base_com_ang_vel(:, 3) = temp;
    rev_joint_ang_velocity = statevar(:, 13 + num_links_without_base :...
        12 + 2 * num_links_without_base);
    num_instants = size(statevar, 1);
    
    % Compute rotation matrix
    rot_mat_link_prev_link_frame = ...
        zeros(3, 3, num_instants, num_links_with_base);
    rot_mat_link = zeros(3, 3, num_instants, num_links_with_base);
    joint_ang_position = [base_sensor_orientation(:,1), rev_joint_ang_position];
    for curr_link = 1 : num_links_with_base
        for curr_instant = 1 : num_instants
            joint_angle = joint_ang_position(curr_instant, curr_link);
            rot_mat_link_prev_link_frame(:, :, curr_instant,...
                curr_link) = [cos(joint_angle) -sin(joint_angle) 0
                              sin(joint_angle)  cos(joint_angle) 0
                              0                 0                1];
        end
    end
    
    rot_mat_link(:, :, :, 1) = rot_mat_link_prev_link_frame(:, :, :, 1);
    start_end_link_index = [arm_initial_link_index, num_links_without_base];
    for curr_instant = 1 : num_instants
        for curr_arm_index = 1 : num_arms
            for link_index_in_robot = start_end_link_index(curr_arm_index) : ...
                    start_end_link_index(curr_arm_index + 1)
                if link_index_in_robot ==  start_end_link_index(curr_arm_index)
                    rot_mat_prev = rot_mat_link(:, :, curr_instant, 1);
                else
                    rot_mat_prev = rot_mat_link(:, :, curr_instant,...
                        link_index_in_robot);
                end
                rot_mat_link(:, :, curr_instant, link_index_in_robot + 1) = ...
                    rot_mat_prev * rot_mat_link_prev_link_frame(link_index_in_robot + 1);
            end
        end
    end
                
    % Compute joint position vector
    joint_position = zeros(3, 1, num_instants, num_links_with_base);
    joint_position(:, :, :, 1) = base_sensor_position.';
    base_arm_joint_to_base_sensor = zeros(3, 1, num_instants, curr_arm_index);
    
    for curr_instant = 1 : num_instants
        for curr_arm_index = 1 : num_arms
            base_arm_joint_to_base_sensor(:, :, curr_instant, curr_arm_index) ...
                = rot_mat_link(:, :, curr_instant, 1) * ...
                base_arm_joint_position_sensor_frame(:, curr_arm_index);
            for link_index_in_robot = start_end_link_index(curr_arm_index) : ...
                    start_end_link_index(curr_arm_index + 1)
                if link_index_in_robot ==  start_end_link_index(curr_arm_index)
                    curr_joint_prev_joint_position = ...
                        base_arm_joint_to_base_sensor(:, :, curr_instant, curr_arm_index);
                    prev_joint_position = joint_position(:, :, curr_instant, 1);
                else
                    curr_joint_prev_joint_position = ...
                        rot_mat_link(:, :, curr_instant, link_index_in_robot) * ...
                        ([link_length(link_index_in_robot + 1), 0, 0].');
                    prev_joint_position = ...
                        joint_position(:, :, curr_instant, link_index_in_robot);
                end
                joint_position(:, :, curr_instant, link_index_in_robot + 1) = ...
                    prev_joint_position + curr_joint_prev_joint_position;
            end
        end
    end
                
    
    reg_mat = 0;
        


end

