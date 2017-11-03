clear all; close all; clc

% Verify trajectory.m
% dbstop in trajectory.m at 8
global joint_pos joint_vel joint_acc
joint_pos = []; joint_vel = []; joint_acc = [];
tr_par_seed = [-1.22313890076491,0.303451868445267,3.20179188236482,0.108153706584789,0.153910196489147,0.0440378283370745,0.135180645621238,0.0440693325406989];
num_arm_joints = 4;
num_rw_joints = 1;
rw_seed_params = [0, 2*pi/3];
tr_par_seed = [tr_par_seed(1 : num_arm_joints), rw_seed_params(1), tr_par_seed(num_arm_joints + 1 : end), rw_seed_params(2)];
num_joints = num_arm_joints + num_rw_joints;
t_diff = 0.05;
% load pruned_data.mat
% tf=numel(pruned_interval_index);
tf=16;

total_time = 0 : t_diff : tf;
num_instants = length(total_time);
vel_combi_mat = generate_vel_combi_mat(num_joints, 1);
unordered_jt_index = 1 : num_arm_joints;
ordered_jt_index = [1 3 2 4];
vel_combi_mat(:, unordered_jt_index) = vel_combi_mat(:, ordered_jt_index);
% vel_combi_mat = vel_combi_mat(pruned_interval_index, :);

num_intervals_each_joint = size(vel_combi_mat, 1);
tr_par = make_tr_params(tr_par_seed, vel_combi_mat);

position = zeros(num_joints, num_instants); 
velocity = zeros(num_joints, num_instants); 
acceleration = zeros(num_joints, num_instants); 

for time_index = 1 : 1 : num_instants
    t = total_time(time_index);
    [position(1 : num_joints, time_index), velocity(1 : num_joints, time_index), ... 
        acceleration(1 : num_joints, time_index)] = trajectory(t, num_joints + 1, tf, tr_par, num_intervals_each_joint);
end

[min_jt_angle, max_jt_angle, min_jt_velocity, max_jt_velocity, ...
                                   min_jt_acc, max_jt_acc] = joint_limits();

kin_data_name = {' Position', ' Velocity', ' Acceleration'};
for curr_joint = 1 : num_joints
    figure();
    for plot_index = 1 : 3
        subplot(3, 1, plot_index);
        if plot_index == 1
            kin_data = position;
%             ll = (180 / pi) * min_jt_angle(curr_joint) * ones(1,  num_instants);
%             ul = (180 / pi) * max_jt_angle(curr_joint) * ones(1,  num_instants);
        elseif plot_index == 2
            kin_data = velocity;
%             ll = (180 / pi) * min_jt_velocity(curr_joint) * ones(1,  num_instants);
%             ul = (180 / pi) * max_jt_velocity(curr_joint) * ones(1,  num_instants);
        else
            kin_data = acceleration;
%             ll = (180 / pi) * min_jt_acc(curr_joint) * ones(1,  num_instants);
%             ul = (180 / pi) * max_jt_acc(curr_joint) * ones(1,  num_instants);
        end
        plot(total_time, (180 / pi) * kin_data(curr_joint, :)); hold on;
%         plot(total_time, ll, 'r');
%         plot(total_time, ul, 'r');
        title(strcat('Joint-', num2str(curr_joint), kin_data_name{plot_index}));
        grid on;
    end
end                               
