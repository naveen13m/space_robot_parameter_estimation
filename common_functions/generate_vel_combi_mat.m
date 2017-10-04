% Constructs the velocity combination matrix for a generic tree-type
% spatial system. Reordering to provide wider range to proximal joints is
% done in the robot's initials.m file
% Inputs: Number of joints excluding the joints of the reaction wheels (1 X 1)
%         Is the system planar (1 X 1)
% Ouputs: Velocity combination matrix (2^(n-1) X n), where n is the number
%         of joints including those of the reaction wheel
function vel_combi_mat = generate_vel_combi_mat(num_arm_joints, is_planar)
    if is_planar
        num_static_rw_cols = 0;
    else
        num_static_rw_cols = 1;
    end
    num_intervals_per_joint = 2 ^ (num_arm_joints - 1);
    vel_combi_mat = zeros(num_intervals_per_joint * 2, num_arm_joints);
    ones_length = num_intervals_per_joint;
    zeros_length = ones_length;
    for curr_joint = 1 : num_arm_joints
        multiplicity = 2^(curr_joint-1);
        vel_combi_mat(:, curr_joint) = repmat([ones(ones_length, 1); zeros(zeros_length, 1)], multiplicity, 1);
        ones_length = ones_length / 2;
        zeros_length = ones_length;
    end 
    vel_combi_mat = [vel_combi_mat(1 : num_intervals_per_joint, 1 : end - 1), ...
                     ones(num_intervals_per_joint, num_static_rw_cols), ...
                     vel_combi_mat(1 : num_intervals_per_joint, end)];
end

