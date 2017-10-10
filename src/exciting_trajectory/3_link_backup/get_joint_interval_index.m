function interval_index = get_joint_interval_index(curr_time, time_params)
    if curr_time > sum(time_params)
        error('Current time exceeds total time!');
    end
    cummulative_time = time_params(1);
    interval_index = 1;
    while curr_time > cummulative_time
        interval_index = interval_index + 1;
        cummulative_time = cummulative_time + time_params(interval_index);
    end
end