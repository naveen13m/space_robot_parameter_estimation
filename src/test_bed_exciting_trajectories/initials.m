function [y0, t_initial, t_final, incr, rtol, atol, vel_combi_mat]=initials()


ordered_jt_index = [1 3 2 4];

%Simulation time
t_initial=0;

%Inverse kinematics for obtaining initial configuration
[num_links, not_planar] = inputs();
is_planar = 1 - not_planar;
if is_planar
    num_rw_joints = 1;
else
    num_rw_joints = 4;
end

% Before pruning
t_final=16;

% % After pruning
% t_final=numel(pruned_interval_index);
% load pruned_data.mat

%Base motions
q=[0; 0; 0; 0; 0; 0];
dq=[0; 0; 0; 0; 0; 0];

% Actuator energy
acten=0;

%Vecotor of all the initial State Variable
y0=[q; dq; acten];

%INTERATION TOLERANCES
incr=0.2;
rtol=1e-5;         %relative tolerance in integration 
atol=1e-7;         %absolute tolerances in integration 

num_arm_joints = num_links - 1 - num_rw_joints;
num_joints = num_links - 1;
vel_combi_mat = generate_vel_combi_mat(num_joints, is_planar);
unordered_jt_index = 1 : num_arm_joints;
vel_combi_mat(:, unordered_jt_index) = vel_combi_mat(:, ordered_jt_index);
% vel_combi_mat = vel_combi_mat(pruned_interval_index, :);
end