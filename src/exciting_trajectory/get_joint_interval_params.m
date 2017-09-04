function [thi, thf, curr_time] = get_joint_interval_params(tr_par, ...
    num_intervals_each_joint, curr_time, Tp, curr_joint)
    num_terms_each_joint = num_intervals_each_joint + 1;
    num_terms_till_curr_joint = num_terms_each_joint * (curr_joint - 1);
    time_params = repmat(Tp, 1, num_intervals_each_joint);
    interval_index = get_joint_interval_index(curr_time, time_params);
    thi = tr_par(num_terms_till_curr_joint + interval_index);
    thf = tr_par(num_terms_till_curr_joint + interval_index + 1);
    curr_time = curr_time - (interval_index - 1) * Tp;
end