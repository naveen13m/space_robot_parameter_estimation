function linear_dependency_verification(num_links_with_base, num_instants)

    link_frame_position = ndSym('r',[2, 1, num_instants]);
    link_frame_position = [link_frame_position; ...
        sym(zeros(1, 1, num_instants))];
    link_frame_position = [link_frame_position, ...
        sym(zeros(3, num_links_with_base - 1, num_instants))];
    
    link_frame_velocity = ndSym('v',[2 1 num_instants]);
    link_frame_velocity = [link_frame_velocity, ...
        zeros(2, num_links_with_base - 1, num_instants)];
    link_frame_velocity = [link_frame_velocity; ...
        zeros(1, num_links_with_base, num_instants)];
    
    joint_position = ndSym('th',[1 num_links_with_base num_instants]);
    joint_speed = ndSym('thd',[1 num_links_with_base num_instants]);
    
    joint_direction = [zeros(2, num_links_with_base); ...
        ones(1, num_links_with_base)];
    
    link_ang_vel = [sym(zeros(2,num_links_with_base, num_instants)); ...
        joint_speed(1,1,:), zeros(1,num_links_with_base-1, num_instants)];
    
    link_frame_prev_link_frame = sym('a',[2,num_links_with_base]);
    link_frame_prev_link_frame = [link_frame_prev_link_frame; ...
        sym(zeros(1,num_links_with_base))];

    % Rotation Matrices
    rot_mat_link_prev_link_frame = sym(zeros(3,3,num_instants,num_links_with_base));
    rot_mat_link = sym(zeros(3,3,num_instants,num_links_with_base));
    for curr_instant = 1 : num_instants
        angle = joint_position(1,1,curr_instant);
        rot_mat_link_prev_link_frame(:, :, curr_instant, 1) = ...
            [cos(angle) -sin(angle)  0
             sin(angle)  cos(angle)  0
             0           0           1];
        rot_mat_link(:, :, curr_instant, 1) = ...
            rot_mat_link_prev_link_frame(:, :, curr_instant, 1);
        for curr_link_index = 2 : num_links_with_base
            angle = joint_position(1, curr_link_index, curr_instant);
            rot_mat_link_prev_link_frame(:, :, curr_instant, curr_link_index) = ...
                [cos(angle) -sin(angle)  0
                 sin(angle)  cos(angle)  0
                 0           0           1];
            rot_mat_link(:, :, curr_instant, curr_link_index) = ...
                simplify(rot_mat_link(:, :, curr_instant,curr_link_index - 1) * ...
                rot_mat_link_prev_link_frame(:, :, curr_instant, curr_link_index));
        end
    end

    % Link frame origin position vectors
    for curr_instant = 1 : num_instants
        for curr_link_index = 2 : num_links_with_base
            link_frame_position(:, curr_link_index, curr_instant) = ...
                link_frame_position(:, curr_link_index-1, curr_instant) + ...
                rot_mat_link(:, :, curr_instant,curr_link_index - 1) * ...
                link_frame_prev_link_frame(:, curr_link_index - 1);
        end
    end
    
    % CoM angular velocity
    ang_vel_mat = sym(zeros(3,3,num_instants,num_links_with_base));
    for curr_instant = 1 : num_instants
        ang_vel_mat(:, :, curr_instant, 1) = ...
            vec_to_mat(link_ang_vel(:, 1, curr_instant));
        for curr_link_index = 2 : num_links_with_base
            link_ang_vel(:, curr_link_index, curr_instant) = ...
                link_ang_vel(:, curr_link_index-1, curr_instant) + ...
                joint_speed(:, curr_link_index, curr_instant) * ...
                joint_direction(:, curr_link_index);
            ang_vel_mat(:, :, curr_instant, curr_link_index) = ...
                vec_to_mat(link_ang_vel(:, curr_link_index, curr_instant));
        end
    end

    % CoM linear velocity
    for curr_instant = 1 : num_instants
        for curr_link_index = 2 : num_links_with_base
            link_frame_velocity(:, curr_link_index, curr_instant) = ...
                simplify(link_frame_velocity(:, curr_link_index-1, curr_instant) + ...
                ang_vel_mat(:, :, curr_instant, curr_link_index-1) * ...
                (link_frame_position(:, curr_link_index, curr_instant) - ...
                link_frame_position(:, curr_link_index-1, curr_instant)));
        end
    end
    
    % Linear momentum regressor matrix
    sym_reg_mat =  sym(zeros(3 * num_instants, 4 * num_links_with_base));
    for curr_instant = 1 : num_instants
        for curr_link_index = 1 : num_links_with_base
            link_reg_mat(:, 4 * curr_link_index - 3 : 4 * curr_link_index) = ...
                [link_frame_velocity(:,curr_link_index,curr_instant), ...
                ang_vel_mat(:, :, curr_instant, curr_link_index) * ...
                rot_mat_link(:, :, curr_instant, curr_link_index)];
        end
        sym_reg_mat(3 * curr_instant - 2 : 3 * curr_instant, :) = link_reg_mat;
    end
    curr_instant = 1 : num_instants;
    sym_reg_mat(3 * curr_instant, :) = [];
    curr_link_index = 1 : num_links_with_base;
    sym_reg_mat(:, 4 * curr_link_index) = [];
    sym_reg_mat_rank = rank(sym_reg_mat)
end

function skew_symm_matrix = vec_to_mat(vector)
    skew_symm_matrix = [0          -vector(3)  vector(2); ...
                        vector(3)   0         -vector(1); ...
                       -vector(2)  vector(1)  0];
end