function [th_d, dth_d, ddth_d]=trajectory(t, n, tf, tr_par, num_intervals_each_joint)
    th_d = zeros(n-1,1); dth_d = zeros(n-1,1); ddth_d = zeros(n-1,1);
    num_joints = n - 1;
    Tp = tf / num_intervals_each_joint;
    for curr_joint = 1 : num_joints
        [thi, thf, t_interval] = get_joint_interval_params(tr_par, ...
            num_intervals_each_joint, t, Tp, curr_joint);
        [th_d(curr_joint, 1), dth_d(curr_joint, 1), ddth_d(curr_joint, 1)] = ...
            cycloidal_traj(thi, thf, t_interval, Tp);
    end
end