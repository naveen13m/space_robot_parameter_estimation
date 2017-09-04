function minimal_param_vec = compute_minimal_param_vector(robot_make, base_sensor_base_frame_position_base_frame)
    curr_dir = pwd;
    cd(strcat('../test_case_data', robot_make, '/config_files'));
    [num_links, not_planar, joint_twist, link_length_DH, ...
        joint_offset, parent_link_index, link_x_com, link_y_com, link_z_com, ...
        ~, ~, link_mass, ~, inertia_xx_com, inertia_yy_com, inertia_zz_com, ...
        inertia_xy_com, inertia_yz_com, inertia_zx_com] = inputs();
    is_planar = 1 - not_planar;
    cd(curr_dir);
    link_x_com(1) = -base_sensor_base_frame_position_base_frame(1);
    link_y_com(1) = -base_sensor_base_frame_position_base_frame(2);
    link_z_com(1) = -base_sensor_base_frame_position_base_frame(3);
    decoupled_param_vec = construct_dyn_param_vector(...
        inertia_xx_com, inertia_yy_com, inertia_zz_com, inertia_xy_com, inertia_yz_com,...
        inertia_zx_com, link_mass, link_x_com, link_y_com, link_z_com, num_links);
    minimal_param_vec = decoupled_param_vec;
    
    arm_initial_link_index = find(parent_link_index == 1);
    num_arms = length(arm_initial_link_index);
    arm_terminal_link_index = [arm_initial_link_index(2 : end) - 1, num_links];
    
    % Backward iteration of link parameters
    eliminate_elements_index = [];
    for curr_arm_index = 1 : num_arms
        arm_link_indices = arm_terminal_link_index(curr_arm_index) : -1 : ...
            arm_initial_link_index(curr_arm_index);
        Iyy_indices = (arm_initial_link_index(curr_arm_index) * 10 - 8) : 10 : arm_terminal_link_index(curr_arm_index) * 10 - 8;
        mass_indices = arm_initial_link_index(curr_arm_index) * 10 - 3 : 10 : arm_terminal_link_index(curr_arm_index) * 10 - 3;
        z_com_indices = arm_initial_link_index(curr_arm_index) * 10 : 10 : arm_terminal_link_index(curr_arm_index) * 10;
        eliminate_elements_index = [eliminate_elements_index, Iyy_indices, mass_indices, z_com_indices];
        for curr_link_index = arm_link_indices
            parent_link = parent_link_index(curr_link_index);
            % Compute coefficients for bacward iteration
            link_length = [link_length_DH(curr_link_index);
                         -joint_offset(curr_link_index) * ...
                          sin(joint_twist(curr_link_index)); ...
                          joint_offset(curr_link_index) * ...
                          cos(joint_twist(curr_link_index))];
            if curr_link_index == arm_initial_link_index(curr_arm_index)
                link_length = link_length - base_sensor_base_frame_position_base_frame;
            end
            s_al = sin(joint_twist(curr_link_index));
            c_al = cos(joint_twist(curr_link_index));
            ilisq = inertia_mat_to_vec(-vec_to_mat(link_length)^2);
            ili = link_length;
            lx = link_length(1); ly = link_length(2); lz = link_length(3);
            t1 = [0  -s_al   c_al].';
            t2 = [1, c_al^2, s_al^2, 0, c_al * s_al, 0].';
            t3 = [2 * (lz * c_al - ly * s_al),  2 * lz * c_al, ...
                -2 * ly * s_al,  lx * s_al, lz * s_al - ly * c_al, -lx * c_al].';
            % Backward iteration for mass
            minimal_param_vec(10 * parent_link - 9 : 10 * parent_link, :) = ...
                minimal_param_vec(10 * parent_link - 9 : 10 * parent_link, :) + ...
                minimal_param_vec(10 * curr_link_index - 3) * ...
                [ilisq;
                 1;
                 ili];
            % Backward iteration for z-com coordinate
            minimal_param_vec(10 * parent_link - 9 : 10 * parent_link, :) = ...
                minimal_param_vec(10 * parent_link - 9 : 10 * parent_link, :) + ...
                minimal_param_vec(10 * curr_link_index, :) * ...
                [t3
                 0;
                 t1];
            % Backward iteration for Iyy
            minimal_param_vec([10 * parent_link - 9 : 10 * parent_link - 4, 10 * curr_link_index - 9], :) = ...
                minimal_param_vec([10 * parent_link - 9 : 10 * parent_link - 4, 10 * curr_link_index - 9], :) + ...
                minimal_param_vec(10 * curr_link_index - 8) * ...
                [t2;
                -1];
        end
    end
    % Remove ld column elements
    if is_planar
        base_Iyy_z_com_indices = [2, 10];
        Ixx_index = 1 : 10 : num_links * 10;
        Ixy_index = 4 : 10 : num_links * 10;
        Iyz_index = 5 : 10 : num_links * 10;
        Ixz_index = 6 : 10 : num_links * 10;
        eliminate_elements_index = [eliminate_elements_index, ...
            base_Iyy_z_com_indices, Ixx_index, Ixy_index, Iyz_index, Ixz_index];
        minimal_param_vec(eliminate_elements_index, :) = [];
    else
        minimal_param_vec(eliminate_elements_index, :) = [];
    end 
end