clear all; close all; clc;

num_arm_joints = 6;
num_rw_joints = 4;
num_joints = num_arm_joints + num_rw_joints;

load('timevar_1.dat');
statevar_1 = load('statevar_1.dat');
statevar_2 = load('statevar_2.dat');
rms_error_pose = [];
rms_error_twist = [];
rms_error_torque = [];
figure(1);
set(gcf,'units','pixels','Position',[0, 0, 1366, 768]); 
plot_type = {'b', 'k--', 'm-.'};

for curr_dir = 1 : 6
    % Base pose
    figure(1)
    subplot(2, 2, fix((curr_dir - 1) / 3) + 1);
    plot(timevar_1, (statevar_1(:, curr_dir) - statevar_2(:, curr_dir)), plot_type{mod(curr_dir - 1, 3) + 1});
    hold on;
    rms_error_pose = [rms_error_pose, rms(statevar_1(:, curr_dir) - statevar_2(:, curr_dir))];
    % Base twist
    subplot(2, 2, fix((curr_dir - 1) / 3) + 3);
    curr_col = 6 + curr_dir + num_joints;
    plot(timevar_1, (statevar_1(:, curr_col) - statevar_2(:, curr_col)), plot_type{mod(curr_dir - 1, 3) + 1});
    hold on;
    rms_error_twist = [rms_error_twist, rms(statevar_1(:, curr_col) - statevar_2(:, curr_col))];
end
figure(1);
subplot(2, 2, 1);
xlabel('time [s]'); ylabel('Position error [m]');
legend('X', 'Y', 'Z', 'Location', 'SouthWest');

subplot(2, 2, 2);
xlabel('time [s]'); ylabel('Euler angle error [rad]');
legend('Z', 'X', 'Y', 'Location', 'SouthWest');

subplot(2, 2, 3);
xlabel('time [s]'); ylabel('Linear velocity error [m/s]');
legend('X', 'Y', 'Z', 'Location', 'SouthWest');

subplot(2, 2, 4);
xlabel('time [s]'); ylabel('Angular velocity error [rad/s]');
legend('X', 'Y', 'Z', 'Location', 'SouthWest');

% Joint torque
tor_1 = load('tor_1.dat');
tor_2 = load('tor_2.dat');
figure(2);
set(gcf,'units','pixels','Position',[0, 0, 1366, 768]);

for curr_joint = 1 : num_arm_joints
    subplot(2, 2, fix((curr_joint - 1) / 3) + 1);
    plot(timevar_1, tor_1(:, curr_joint) - tor_2(:, curr_joint), plot_type{mod(curr_joint - 1, 3) + 1});
    hold on;
    rms_error_torque = [rms_error_torque, rms(tor_1(:, curr_joint) - tor_2(:, curr_joint))];
end

figure(2);
subplot(2, 2, 1);
xlabel('time [s]'); ylabel('Torque error [m]');
axis([-Inf Inf -0.3 0.12]);
legend('\tau_{1}', '\tau_{2}', '\tau_{3}', 'Location', 'SouthWest');

subplot(2, 2, 2);
xlabel('time [s]'); ylabel('Torque error [rad]');
axis([-Inf Inf -1 0.7]);
legend('\tau_{4}', '\tau_{5}', '\tau_{6}', 'Location', 'SouthWest');
