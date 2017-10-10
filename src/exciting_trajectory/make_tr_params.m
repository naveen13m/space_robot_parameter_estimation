function tr_par_0 = make_tr_params(tr_par_seed, vel_combi_mat)
    [num_intervals_each_joint, num_joints] = size(vel_combi_mat);
    num_params = (num_intervals_each_joint + 1) * num_joints;
    tr_par_0 = zeros(num_intervals_each_joint + 1, num_joints);
    
    lnt = length(tr_par_seed);
    start_position = [tr_par_seed(1 : lnt / 2), 0, 0, pi/2, 0];
    coeff_increment = [tr_par_seed(lnt / 2 + 1 : end), 3*pi, 3*pi, 0, 3*pi];  
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
%     figure();
%     k = 180 / pi;
%     subplot(3, 3, 1)
%     plot(tr_par_0(1 : 33) * k)
%     subplot(3, 3, 2)
%     plot(tr_par_0(34 : 66) * k)
%     subplot(3, 3, 3)
%     plot(tr_par_0(67 : 99) * k)
%     subplot(3, 3, 4)
%     plot(tr_par_0(100 : 132) * k)
%     subplot(3, 3, 5)
%     plot(tr_par_0(133 : 165) * k)
%     subplot(3, 3, 6)
%     plot(tr_par_0(166 : 198) * k)
%     subplot(3, 3, 7)
%     plot(tr_par_0(199 : 231) * k)
%     subplot(3, 3, 8)
%     plot(tr_par_0(232 : 264) * k)
%     subplot(3, 3, 9)
%     plot(tr_par_0(265 : 297) * k)
end

