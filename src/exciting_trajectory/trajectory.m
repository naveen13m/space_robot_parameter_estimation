function [th_d, dth_d, ddth_d]=trajectory(t, n, tf, tr_par)
    th_d = zeros(n-1,1); dth_d = zeros(n-1,1); ddth_d = zeros(n-1,1);
    num_joints = n - 1;
    num_intervals = 2 ^ (num_joints - 1);
    Tp = tf / num_intervals;
    for curr_joint = 1 : num_joints
        [thi, thf, t_interval] = get_joint_interval_params(tr_par, t, Tp, curr_joint, num_joints);
        [th_d(curr_joint, 1), dth_d(curr_joint, 1), ddth_d(curr_joint, 1)] = cycloidal_traj(thi, thf, t_interval, Tp);
    end
end