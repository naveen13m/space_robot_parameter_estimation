clear all; close all; clc

% Verify trajectory.m
tr_par = [0.5 pi/20, -0.12 -0.05  0.1  -0.05  0.02, ... 
                             -0.62 -0.05 -0.15  0.2  -0.03, ...
                 0.3, zeros(1, 11), ...
                 0    pi/20,  0.23 -0.03 -0.15  0.2  -0.03, ...
                             -1.13  0.09  0.1  -0.05  0.02];
       
num_joints = 3;
t_diff = 0.1;
total_time = 0 : t_diff : 20;
num_instants = length(total_time);
fourier_value = zeros(num_joints, num_instants); 
dfourier_value = zeros(num_joints, num_instants); 
ddfourier_value = zeros(num_joints, num_instants); 

[min_jt_angle, max_jt_angle, min_jt_vel, max_jt_vel, ...
                                   min_jt_acc, max_jt_acc] = joint_limits();
                               
for time_index = 1 : 1 : num_instants
    [fourier_value(1 : num_joints, time_index), dfourier_value(1 : num_joints, time_index), ... 
        ddfourier_value(1 : num_joints, time_index)] = ...
        trajectory(total_time(time_index), num_joints + 1, tf, tr_par);
end

subplot(1, 3, 1);
plot(total_time, fourier_value(1, :), total_time, fourier_value(2, :), ...
    total_time, fourier_value(3, :)); hold on;
plot(total_time, min_jt_angle(1) * ones(1, num_instants), ...
    total_time, min_jt_angle(2) * ones(1, num_instants));
plot(total_time, max_jt_angle(1) * ones(1, num_instants), ...
    total_time, max_jt_angle(2) * ones(1, num_instants))
legend('jt-1','jt-2', 'jt-3', 'jt1-ll', 'jt2-ll', 'jt1-ul', 'jt2-ul');

subplot(1, 3, 2);
plot(total_time, dfourier_value(1, :), total_time, dfourier_value(2, :), ...
    total_time, dfourier_value(3, :)); hold on;
plot(total_time, min_jt_vel(1) * ones(1, num_instants), ...
    total_time, min_jt_vel(2) * ones(1, num_instants), ...
    total_time, min_jt_vel(2) * ones(1, num_instants));
plot(total_time, max_jt_vel(1) * ones(1, num_instants), ...
    total_time, max_jt_vel(2) * ones(1, num_instants), ...
    total_time, max_jt_vel(2) * ones(1, num_instants))
legend('jt-1','jt-2', 'jt-3' , 'jt1-ll', 'jt2-ll', 'jt3-ll' , 'jt1-ul', 'jt2-ul', 'jt3-ul');

subplot(1, 3, 3);
plot(total_time, ddfourier_value(1, :), total_time, ddfourier_value(2, :), ...
    total_time, ddfourier_value(3, :)); hold on;
plot(total_time, min_jt_acc(1) * ones(1, num_instants), ...
    total_time, min_jt_acc(2) * ones(1, num_instants), ...
    total_time, min_jt_acc(2) * ones(1, num_instants));
plot(total_time, max_jt_acc(1) * ones(1, num_instants), ...
    total_time, max_jt_acc(2) * ones(1, num_instants), ...
    total_time, max_jt_acc(2) * ones(1, num_instants));
legend('jt-1','jt-2', 'jt-3' , 'jt1-ll', 'jt2-ll', 'jt3-ll' , 'jt1-ul', 'jt2-ul', 'jt3-ul');