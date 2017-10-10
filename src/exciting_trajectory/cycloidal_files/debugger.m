clear all; close all; clc

% Verify trajectory.m
% dbstop in trajectory.m at 8
vel_combi_mat = generate_vel_combi_mat(6);
tr_par_seed = [-1.57079632679490,-0.872664625997165,-1.52994040764316,-0.399601038688018,-1.69032200813286,0.628260324627678,0.0981747704246810,0.174532925199433,0.174532925199433,0.174532925199433,0.174532925199433,0.174532925199433];
tr_par = make_tr_params(tr_par_seed, vel_combi_mat);

num_joints = 6;
t_diff = 0.2;
tf = 64;
total_time = 0 : t_diff : tf;
num_instants = length(total_time);

vel_combi_mat = generate_vel_combi_mat(num_joints);
num_intervals_each_joint = size(vel_combi_mat, 1);
% tr_par = guess_initial_param(vel_combi_mat);


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
                               
for plot_index = 1 : 3
    subplot(3, 1, plot_index);
    if plot_index == 1
        data = pos;
        ll = min_jt_angle * ones(1,  num_instants);
        ul = max_jt_angle * ones(1,  num_instants);
    elseif plot_index == 2
        data = vel;
        ll = min_jt_velocity * ones(1,  num_instants);
        ul = max_jt_velocity * ones(1,  num_instants);
    else
        data = acc;
        ll = min_jt_acc * ones(1,  num_instants);
        ul = max_jt_acc * ones(1,  num_instants);
    end
    for curr_joint = 1 : num_joints
        plot(total_time, data(curr_joint, :)); hold on;
        plot(total_time, ll, 'r');
        plot(total_time, ul, 'r');
    end
%     legend('jt-1', [], [],  'jt-2', [], []);
    grid on;
end