% Computes the joint angle, joint angular rate and rate of angular rate for
% all the joints a gven instant based on interwise cycloidal
% parametrization.
% Inputs: Current time (1 X 1)
%         Number of joints including those of reaction wheeel(1 X 1)
%         Total time for experiment (1 X 1)
%         Interval trajectory parameters (Output of make_tr_params)
%         Number of intervals (Number of rows of velocity combination matrix)
function [th_d, dth_d, ddth_d]=trajectory(t, n, tf, tr_par, num_intervals_each_joint)
    global joint_pos joint_vel joint_acc
    th_d = zeros(n-1,1); dth_d = zeros(n-1,1); ddth_d = zeros(n-1,1);
    num_joints = n - 1;
    Tp = tf / num_intervals_each_joint;
    for curr_joint = 1 : num_joints
        [thi, thf, t_interval] = get_joint_interval_params(tr_par, ...
            num_intervals_each_joint, t, Tp, curr_joint);
        if isempty(t_interval)
            t_interval = t_first_joint;
        else
            t_first_joint = t_interval;
        end
        [th_d(curr_joint, 1), dth_d(curr_joint, 1), ddth_d(curr_joint, 1)] = ...
            cycloidal_traj(thi, thf, t_interval, Tp);
    end
end