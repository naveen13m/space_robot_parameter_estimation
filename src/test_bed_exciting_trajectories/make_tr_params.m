% Generates the parameters of all the cycloidal trajectories in each
% interval.
% Input: Trajectory seed parameters (1 X 2n, n = num of joints of the robotic arms)
%        Velocity combination matrix (a X b, a = num of intervals, b = num of joints in the system)
% Output: Parameters of the cycloidal trajectory of all the joints for
% every interval ((a+1) X b)
function tr_par_0 = make_tr_params(tr_par_seed, vel_combi_mat)
    [num_intervals_each_joint, num_joints] = size(vel_combi_mat);
    num_params = (num_intervals_each_joint + 1) * num_joints;
    tr_par_0 = zeros(num_intervals_each_joint + 1, num_joints);
    
    lnt = length(tr_par_seed);
    start_position = [tr_par_seed(1 : lnt / 2), 0];
    coeff_increment = [tr_par_seed(lnt / 2 + 1 : end), 2*pi/3];  
    for curr_joint_index = 1 : num_joints
        tr_par_0(1, curr_joint_index) = start_position(curr_joint_index);
        for curr_coeff_index = 2 : num_intervals_each_joint + 1
            curr_interval_index = curr_coeff_index - 1;
            if vel_combi_mat(curr_interval_index, curr_joint_index) == 1
                tr_par_0(curr_coeff_index, curr_joint_index) = ...
                    tr_par_0(curr_coeff_index - 1, curr_joint_index) + ...
                        coeff_increment(curr_joint_index);
            else
                tr_par_0(curr_coeff_index, curr_joint_index) = ...
                    tr_par_0(curr_coeff_index - 1, curr_joint_index) - ...
                        coeff_increment(curr_joint_index);
            end
        end
    end
    tr_par_0 = reshape(tr_par_0, 1, num_params);
end

