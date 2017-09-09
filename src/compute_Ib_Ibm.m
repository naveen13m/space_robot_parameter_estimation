function [Ib, Ibm] = compute_Ib_Ibm(base_pose, joint_position, num_links, ...
                                joint_twist, link_length_DH, joint_offset, ...
                                parent_link_index, link_x_com, link_y_com, ...
                                link_z_com, link_mass, Icxx, Icyy, Iczz, Icxy, ...
                                Icyz, Iczx, base_arm_joint_sensor_frame_position_sensor_frame)
       
    % Data assignment
    arm_initial_link_index = find(parent_link_index == 1);
    arm_terminal_link_index = [arm_initial_link_index(2 : end) - 1, num_links];
    num_arms = length(arm_initial_link_index);
    num_joints = num_links - 1;
        
    base_sensor_position = base_pose(1 : 3);
    base_sensor_orientation = base_pose(4 : 6); % ZXY order
    
    % Regressor matrix data initialization
    rot_mat_link_prev_link_frame = zeros(3, 3, num_links);
    rot_mat_link = zeros(3, 3, num_links);
    link_frame_position = zeros(3, num_links);
    
    % Compute orientation of link frames - Rotation matrix
    curr_link = 1;
    phi = base_sensor_orientation(1);
    theta = base_sensor_orientation(2);
    psi = base_sensor_orientation(3);
    rot_mat_link_prev_link_frame(:, :, curr_link) = ...
        ZXY_euler_to_rot_mat(phi, theta, psi);
    rot_mat_link(:, :, 1) = ...
        rot_mat_link_prev_link_frame(:, :, 1);
    for curr_link = 2 : num_links
        curr_joint_index = curr_link - 1;
        curr_joint_angle = joint_position(curr_joint_index);
        curr_joint_twist = joint_twist(curr_link);
        rot_mat_link_prev_link_frame(:, :, curr_link) = ...
            DH_to_rot_mat(curr_joint_twist, curr_joint_angle);
        pli = parent_link_index(curr_link);
        rot_mat_parent_link = rot_mat_link(:, :, pli);
        rot_mat_link(:, :, curr_link) = ...
            rot_mat_parent_link * ...
            rot_mat_link_prev_link_frame(:, :, curr_link);
        rot_mat_link(:, :, curr_link);
    end

    
    % Compute link frame position vector
    link_frame_position(:, 1) = base_sensor_position;
    base_arm_joint_base_sensor_position = zeros(3, num_arms);
    for curr_arm = 1 : num_arms
        base_arm_joint_base_sensor_position(:, curr_arm) ...
            = rot_mat_link(:, :, 1) * ...
            base_arm_joint_sensor_frame_position_sensor_frame(:, curr_arm);
        for curr_link = arm_initial_link_index(curr_arm) : ...
                arm_terminal_link_index(curr_arm)
            if curr_link ==  arm_initial_link_index(curr_arm)
                curr_link_frame_prev_link_frame_position = ...
                    base_arm_joint_base_sensor_position(:, curr_arm);
                prev_link_frame_position = link_frame_position(:, 1);
            else
                curr_link_frame_prev_link_frame_position_prev_link_frame = ...
                    [link_length_DH(curr_link);
                    -joint_offset(curr_link) * ...
                    sin(joint_twist(curr_link)); ...
                    joint_offset(curr_link) * ...
                    cos(joint_twist(curr_link))];
                curr_link_frame_prev_link_frame_position = ...
                    rot_mat_link(:, :, curr_link - 1) * ...
                    curr_link_frame_prev_link_frame_position_prev_link_frame;
                prev_link_frame_position = ...
                    link_frame_position(:, curr_link - 1);
            end
            link_frame_position(:, curr_link) = ...
                prev_link_frame_position + curr_link_frame_prev_link_frame_position;
        end
    end
    
    % Compute com frame position vector in base frame
    link_com = zeros(3, num_links);
    for curr_link = 1 : num_links
        link_com_link_frame_position = rot_mat_link(:, :, curr_link) * ...
            [link_x_com(curr_link); link_y_com(curr_link); link_z_com(curr_link)];
        link_com(:, curr_link) = ...
             link_frame_position(:, curr_link) + link_com_link_frame_position;
    end
    sys_com = link_com * link_mass / sum(link_mass);
    
    sys_com_base_frame_position = sys_com - link_frame_position(:, 1);
    link_com_base_frame_position = link_com - repmat(link_frame_position(:, 1), 1, num_links);
       
    % Compute r0g_tilde and Hw
    Hw = zeros(3, 3);
    Ii = zeros(3, 3, num_links);
    for curr_link = 1 : num_links
        Ii_cm = [Icxx(curr_link), Icxy(curr_link), Iczx(curr_link);
                 Icxy(curr_link), Icyy(curr_link), Icyz(curr_link);
                 Iczx(curr_link), Icyz(curr_link), Iczz(curr_link)];
        Ii(:, :, curr_link) = rot_mat_link(:, :, curr_link) * Ii_cm * rot_mat_link(:, :, curr_link).';
        r0i_tilde = vec_to_mat(link_com_base_frame_position(:, curr_link));
        Hw = Hw + Ii(:, :, curr_link) + link_mass(curr_link) * (r0i_tilde.' * r0i_tilde);
    end
    M = sum(link_mass);
    r0g_tilde = vec_to_mat(sys_com_base_frame_position);
    
    % Ib assembly
    Ib = [M * eye(3)   , M * r0g_tilde.'; 
          M * r0g_tilde, Hw            ];
    
    % Compute jacobian
    jacob_rot_manip = zeros(3, num_joints, num_joints);
    jacob_trans_manip = zeros(3, num_joints, num_joints);
    for curr_joint = 1 : num_joints
        for iter_joint = 1 : curr_joint
            jacob_rot_manip(:, iter_joint, curr_joint) = rot_mat_link(:, 3, iter_joint + 1);
            jacob_trans_manip(:, iter_joint, curr_joint) = ...
                cross(jacob_rot_manip(:, iter_joint, curr_joint),...
                      (link_com(:, curr_joint + 1) - ...
                      link_frame_position(:, iter_joint + 1)));
        end
    end
    
    % Compute JTg and Hwphi
    JTg = zeros(3, num_joints);
    Hwphi = zeros(3, num_joints);
    for curr_joint = 1 : num_joints
        curr_link = curr_joint + 1;
        JTg = JTg + link_mass(curr_link) * jacob_trans_manip(:, :, curr_joint);
        Hwphi = Hwphi + (Ii(:, :, curr_link) * jacob_rot_manip(:, :, curr_joint) + ...
                        link_mass(curr_link) * ...
                        vec_to_mat(link_com_base_frame_position(:, curr_link)) * ...
                        jacob_trans_manip(:, :, curr_joint));
    end
    
    % Ibm assembly
    Ibm = [JTg;
           Hwphi];
end
