function [reg_mat, rank_reg_mat, reg_mat_modified, ...
    rank_reg_mat_modified] = linear_dependency_verification(num_links, ...
    num_instants, not_planar)
    close all; clc;
    
    % Structural data and sensor kinematic data
    num_links_without_base = num_links - 1;
    link_length_DH = nd_sym('l_', [num_links, 1]).';
    link_length_DH(1) = sym(0);
    if not_planar
        base_frame_position = nd_sym('r_',[3, num_instants]);
        base_frame_orientation = [nd_sym('phi_',[1, num_instants]); ...
            nd_sym('th_',[1, num_instants]); nd_sym('psi_',[1, num_instants])];
        base_frame_lin_velocity = nd_sym('v_',[3, num_instants]);
        base_frame_ang_velocity = [nd_sym('dpsi_',[1, num_instants]); ...
            nd_sym('dth_',[1, num_instants]); nd_sym('dphi_',[1, num_instants])];
        index = floor((10 - 1) .* rand(num_links, 1) + 1);
        joint_twist_domain = [-pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, 0];
        joint_twist_angle = sym(joint_twist_domain(index));
        joint_twist_angle(1) = sym(0);
        joint_offset = nd_sym('b_', [num_links, 1]).';
        joint_offset(1) = sym(0);
    else
        base_frame_position = [nd_sym('r_',[2, num_instants]); sym(zeros(1, num_instants))];
        base_frame_orientation = [nd_sym('phi_',[1, num_instants]); ...
            sym(zeros(1, num_instants)); sym(zeros(1, num_instants))]; %ZXY
        base_frame_lin_velocity = [nd_sym('v_',[2, num_instants]); sym(zeros(1, num_instants))];
        base_frame_ang_velocity = [sym(zeros(1, num_instants)); ...
            sym(zeros(1, num_instants)); nd_sym('dphi_',[1, num_instants])];
        joint_twist_angle = sym(zeros(1, num_links));
        joint_offset = sym(zeros(1, num_links));
    end
    joint_position = nd_sym('j_th_',[num_links_without_base, num_instants]);
    joint_speed = nd_sym('j_thd_',[num_links_without_base, num_instants]);
    
    % Regressor matrix data initialization
    rot_mat_link_prev_link_frame = sym(zeros(3, 3, num_instants, num_links));
    rot_mat_link = sym(zeros(3, 3, num_instants, num_links));
    link_frame_position = sym(zeros(3, num_instants, num_links));
    link_frame_lin_velocity = sym(zeros(3, num_instants, num_links));
    link_frame_ang_velocity = sym(zeros(3, num_instants, num_links));
    
    link_frame_position(:, :, 1) = base_frame_position;
    link_frame_lin_velocity(:, :, 1) = base_frame_lin_velocity;
    link_frame_ang_velocity(:, :, 1) =  base_frame_ang_velocity;
    
    % % Regressor matrix data computation
    % Compute orientation of link frames - Rotation matrix
    for curr_instant = 1 : num_instants
        phi =  base_frame_orientation(1, curr_instant);
        theta =  base_frame_orientation(2, curr_instant);
        psi =  base_frame_orientation(3, curr_instant);
        rot_mat_link_prev_link_frame(:, :, curr_instant, 1) = ...
            ZXY_euler_to_rot_mat(phi, theta, psi);
        rot_mat_link(:, :, curr_instant, 1) = ...
            rot_mat_link_prev_link_frame(:, :, curr_instant, 1);
        for curr_link_index = 2 : num_links
            curr_joint_index = curr_link_index - 1;
            rot_mat_link_prev_link_frame(:, :, curr_instant, curr_link_index) = ...
                DH_to_rot_mat(joint_twist_angle(curr_link_index), joint_position(curr_joint_index, curr_instant));
            rot_mat_link(:, :, curr_instant, curr_link_index) = ...
                simplify(rot_mat_link(:, :, curr_instant, curr_link_index - 1) * ...
                rot_mat_link_prev_link_frame(:, :, curr_instant, curr_link_index));
        end
    end
    
    % Link frame origin position vectors
    for curr_instant = 1 : num_instants
        for curr_link_index = 2 : num_links
            link_frame_prev_link_frame = ...
                [link_length_DH(curr_link_index); ...
                -joint_offset(curr_link_index) * sin(joint_twist_angle(curr_link_index)); ...
                 joint_offset(curr_link_index) * cos(joint_twist_angle(curr_link_index))];
            link_frame_position(:, curr_instant, curr_link_index) = ...
                link_frame_position(:, curr_instant, curr_link_index - 1) + ...
                rot_mat_link(:, :, curr_instant, curr_link_index - 1) * ...
                link_frame_prev_link_frame;
        end
    end
    
    % CoM angular velocity
    for curr_instant = 1 : num_instants
        for curr_link_index = 2 : num_links            
            curr_joint_index = curr_link_index - 1;
            link_frame_ang_velocity(:, curr_instant, curr_link_index) = ...
                link_frame_ang_velocity(:, curr_instant, curr_link_index - 1) + ...
                rot_mat_link(:, :, curr_instant, curr_link_index) * ...
                [0; 0; joint_speed(curr_joint_index, curr_instant)];
        end
    end

    % CoM linear velocity
    for curr_instant = 1 : num_instants
        for curr_link_index = 2 : num_links
            link_frame_lin_velocity(:, curr_instant, curr_link_index) = ...
                link_frame_lin_velocity(:, curr_instant, 1) + ...
                cross(link_frame_ang_velocity(:, curr_instant, 1), ...
                (link_frame_position(:, curr_instant, curr_link_index) - ...
                link_frame_position(:, curr_instant, 1)));
            for iter_link_index = 2 : curr_link_index
                iter_joint_index = iter_link_index - 1;
                link_frame_lin_velocity(:, curr_instant, curr_link_index) = ...
                    link_frame_lin_velocity(:, curr_instant, curr_link_index) + ...
                    cross(rot_mat_link(:, :, curr_instant, iter_link_index) * ...
                    [0; 0; joint_speed(iter_joint_index, curr_instant)], ...
                    (link_frame_position(:, curr_instant, curr_link_index) - ...
                    link_frame_position(:, curr_instant, iter_link_index)));
            end
        end
    end
    
    % Regressor matrix assembly
    num_equs = 6;
    num_link_params = 10;
    reg_mat = sym(zeros(num_equs * num_instants, num_link_params * num_links));
    inertia_reg_mat_link = sym(zeros(3 * num_instants, 6 * num_links));
    for curr_instant = 1 : num_instants
        for curr_link_index = 1 : num_links
            submatrix_11 = zeros(3, 6);
            submatrix_12 = link_frame_lin_velocity(:, curr_instant, curr_link_index);
            ang_vel_matrix = vec_to_mat(link_frame_ang_velocity(:, curr_instant, curr_link_index));
            submatrix_13 = ang_vel_matrix * rot_mat_link(:, :, curr_instant, curr_link_index);
            submatrix_21 = rot_mat_link(:, :, curr_instant, curr_link_index) * ...
                bullet(rot_mat_link(:, :, curr_instant, curr_link_index).' * ...
                link_frame_ang_velocity(:, curr_instant, curr_link_index));
            submatrix_22 = cross(link_frame_position(:, curr_instant, curr_link_index), ...
                submatrix_12);
            submatrix_23 = (vec_to_mat(link_frame_position(:, curr_instant, curr_link_index)) * ...
                vec_to_mat(link_frame_ang_velocity(:, curr_instant, curr_link_index)) - ...
                vec_to_mat(submatrix_12)) * rot_mat_link(:, :, curr_instant, curr_link_index);
            link_mat = [submatrix_11, submatrix_12, submatrix_13; ...
                        submatrix_21, submatrix_22, submatrix_23];
            curr_instant_reg_mat(:, num_link_params * (curr_link_index - 1) + 1 :...
                num_link_params * curr_link_index) = link_mat;
            inertia_reg_mat_link_instant(:, 6 * (curr_link_index - 1) + 1 :...
                6 * curr_link_index ) = submatrix_21;
        end
        reg_mat(num_equs * (curr_instant - 1) + 1 : num_equs * curr_instant, :) = curr_instant_reg_mat;
        inertia_reg_mat_link(3 * (curr_instant - 1) + 1 : 3 * curr_instant, :) = inertia_reg_mat_link_instant;
    end
%     inertia_reg_mat_link = simplify(inertia_reg_mat_link)
    rank_reg_mat = rank(reg_mat);
    rank_reg_mat_modified = rank(reg_mat);
    
    reg_mat_modified = reg_mat;
    if ~not_planar
        base_link_cols = [1, 2, 4, 5, 6, 10];
        base_link_rows = [3, 4, 5];
        num_cols_delete = length(base_link_cols);
        num_rows_delete = length(base_link_rows);
        cols_to_delete = zeros(1, num_cols_delete * num_links);
        rows_to_delete = zeros(1, num_rows_delete * num_instants);
        for curr_link_index = 1 : num_links
            curr_link_cols = num_link_params * (curr_link_index - 1) + base_link_cols;
            cols_to_delete(:, num_cols_delete * (curr_link_index - 1) + 1 :...
                num_cols_delete * (curr_link_index)) = curr_link_cols;
        end
        for curr_instant = 1 : num_instants
            curr_instant_rows = num_equs * (curr_instant - 1) + base_link_rows;
            rows_to_delete(:, num_rows_delete * (curr_instant - 1) + 1 :...
                num_rows_delete * curr_instant) = curr_instant_rows;
        end
        reg_mat_modified(:, cols_to_delete) = [];
        reg_mat_modified(rows_to_delete, :) = [];
        rank_reg_mat_modified = rank(reg_mat_modified);
    end
end

function s = nd_sym(x,a)
    a = a(:).';
    format = repmat('%d_',[1 numel(a)]);
    x = [x format(1:end-1)];

    s = cellfun(@createCharArrayElement,num2cell(1:prod(a)),'UniformOutput',false);
    s = sym(reshape(s,a));

    function s = createCharArrayElement(k)
        [v{1:numel(a)}] = ind2sub(a,k);
        s = sprintf(x,v{:});
    end
end