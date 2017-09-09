clear all; close all; clc;

% Config params
robot_make = '/dual_arm_articulate';
base_sensor_base_frame_position_base_frame = [-0.2, -0.3, 0].';
base_arm_joint_sensor_frame_position_sensor_frame = [0.7, 0.3, 0.5; -0.3, 0.3, 0.5].';

curr_dir = pwd;
cd(strcat('../test_case_data', robot_make, '/config_files'));
[num_links, ~, joint_twist, link_length_DH, ...
    joint_offset, parent_link_index, link_x_com, link_y_com, link_z_com, ...
    ~, ~, link_mass, ~, inertia_xx_com, inertia_yy_com, inertia_zz_com, ...
    inertia_xy_com, inertia_yz_com, inertia_zx_com] = inputs();
cd(strcat('../sim_real_data/'));
load('statevar.dat'); load('mtvar.dat'); 
cd(curr_dir);

link_x_com(1) = -base_sensor_base_frame_position_base_frame(1);
link_y_com(1) = -base_sensor_base_frame_position_base_frame(2);
link_z_com(1) = -base_sensor_base_frame_position_base_frame(3);

num_instants = size(statevar, 1);
num_joints = num_links - 1;
mtum = zeros(6, num_instants);

for curr_instant = 1 : num_instants
    base_pose = statevar(curr_instant, 1 : 6).';
    joint_position = statevar(curr_instant, 7 : 6 + num_joints).';
    vs = statevar(curr_instant, 7 + num_joints : 9 + num_joints).';
    oms = statevar(curr_instant, 10 + num_joints : 12 + num_joints).';
    dphi = statevar(curr_instant, 13 + num_joints : 12 + 2 * num_joints).';
    
    [Ib, Ibm] = compute_Ib_Ibm(base_pose, joint_position, num_links, ...
                    joint_twist, link_length_DH, joint_offset, ...
                    parent_link_index, link_x_com, link_y_com, ...
                    link_z_com, link_mass, inertia_xx_com, inertia_yy_com, ...
                    inertia_zz_com, inertia_xy_com, inertia_yz_com, ...
                    inertia_zx_com, base_arm_joint_sensor_frame_position_sensor_frame);
    r0 = base_pose(1 : 3);
    P = mtvar(curr_instant, 1 : 3).';
    correction = [zeros(3, 1); cross(r0, P)];
    mtum(:, curr_instant) = Ib * [vs; oms] + Ibm * dphi;
end            

figure();
subplot(2, 3, 1)
plot(mtum(1, :), 'm'); hold on; plot(mtvar(:, 1), 'g--');
subplot(2, 3, 2)
plot(mtum(2, :), 'm'); hold on; plot(mtvar(:, 2), 'g--');
subplot(2, 3, 3)
plot(mtum(3, :), 'm'); hold on; plot(mtvar(:, 3), 'g--');
subplot(2, 3, 4)
plot(mtum(4, :), 'm'); hold on; plot(mtvar(:, 4), 'g--');
subplot(2, 3, 5)
plot(mtum(5, :), 'm'); hold on; plot(mtvar(:, 5), 'g--');
subplot(2, 3, 6)
plot(mtum(6, :), 'm'); hold on; plot(mtvar(:, 6), 'g--');       