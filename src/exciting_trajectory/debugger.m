clear all; close all; clc

% Verify trajectory.m
% dbstop in trajectory.m at 8
global joint_pos joint_vel joint_acc
joint_pos = []; joint_vel = []; joint_acc = [];
% opti_sol = [-1.570796, -1.919862, -0.933581, 1.570796, -1.919862, 0.082142, 0.012274, 0.043892, 0.044846, 0.024567, 0.043554, 0.041980, ]
tr_par_seed = [-1.570796, -1.919862, -0.933581, 1.570796, -1.919862, 0.082142, 0, 0, pi/2, 0, ...
                0.012274, 0.043892, 0.044846, 0.024567, 0.043554, 0.041980, 0, 0, 0, 0];


num_joints = 6 + 3 + 1;
t_diff = 0.2;
tf = 256;
total_time = 0 : t_diff : tf;
num_instants = length(total_time);
vel_combi_mat = generate_vel_combi_mat(num_joints - 1, 0);
unordered_jt_index = 1 : 6;
ordered_jt_index = [1 3 5 2 4 6];
vel_combi_mat(:, unordered_jt_index) = vel_combi_mat(:, ordered_jt_index);
num_intervals_each_joint = size(vel_combi_mat, 1);
tr_par = make_tr_params(tr_par_seed, vel_combi_mat);

pos = zeros(num_joints, num_instants); 
vel = zeros(num_joints, num_instants); 
acc = zeros(num_joints, num_instants); 

% thi = 0;
% thf = pi/2;
for time_index = 1 : 1 : num_instants
    t = total_time(time_index);
    [pos(1 : num_joints, time_index), vel(1 : num_joints, time_index), ... 
        acc(1 : num_joints, time_index)] = trajectory(t, num_joints + 1, tf, tr_par, num_intervals_each_joint);
end

[min_jt_angle, max_jt_angle, min_jt_velocity, max_jt_velocity, ...
                                   min_jt_acc, max_jt_acc] = joint_limits();

for curr_joint = 1 : 6
    figure();
    for plot_index = 1 : 3
        subplot(3, 1, plot_index);
        if plot_index == 1
            data = pos;
            ll = (180 / pi) * min_jt_angle(curr_joint) * ones(1,  num_instants);
            ul = (180 / pi) * max_jt_angle(curr_joint) * ones(1,  num_instants);
        elseif plot_index == 2
            data = vel;
            ll = (180 / pi) * min_jt_velocity(curr_joint) * ones(1,  num_instants);
            ul = (180 / pi) * max_jt_velocity(curr_joint) * ones(1,  num_instants);
        else
            data = acc;
            ll = (180 / pi) * min_jt_acc(curr_joint) * ones(1,  num_instants);
            ul = (180 / pi) * max_jt_acc(curr_joint) * ones(1,  num_instants);
        end
        plot(total_time, (180 / pi) * data(curr_joint, :)); hold on;
%         plot(total_time, ll, 'r');
%         plot(total_time, ul, 'r');
        grid on;
    end
end                               
