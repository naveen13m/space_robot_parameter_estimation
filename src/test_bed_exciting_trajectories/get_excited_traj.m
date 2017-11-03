clear all; close all; clc

% dbstop in runinv.m at 76
% Results verification
global iter fileID cost_value cond_num_reg_mat inverse_signal_strength
global joint_pos joint_vel joint_acc
joint_pos = []; joint_vel = []; joint_acc = [];
iter = 0;
filename = 'opti_data.txt';
fileID = fopen(filename, 'w');

% Config params
% % System configuration
[num_links, not_planar] = inputs();
[~, ~, tf, ~, ~, ~, vel_combi_mat] = initials();
[min_jt_angle, max_jt_angle, min_jt_speed, max_jt_speed, min_jt_acc,...
                                            max_jt_acc] = joint_limits();
rw_params = make_real_reaction_wheel();

is_planar = 1 - not_planar;
if is_planar
    num_rw_joints = 1;              
    base_sensor_base_frame_position_base_frame = [0; 0; 0];
    base_rw_joint_sensor_frame_position_sensor_frame = [0; 0.0196; 0];
end
num_arm_joints = num_links - 1 - num_rw_joints;
num_intervals_each_joint = size(vel_combi_mat, 1);
num_coeffs_each_joint = num_intervals_each_joint + 1;


% % Opti Config
% % % Initial guess, lower and upper bounds of trajectory seed coeffs
tr_par_seed_0 = [-1.22322717975037,0.303486929519048,3.20184440057563,0.108182479039328,...
                0.153915040012465,0.0440391339597696,0.135172339453367,0.0440706391778323];

%                 [-1.5, 0, 3.7, 0.1, ...
%                 0.12, 0.12, -0.12, 0.12];
tr_par_seed_lb = [min_jt_angle.', ...
                  (pi/180 * 0.2) * ones(1, num_arm_joints)];
tr_par_seed_ub = [max_jt_angle.', ...
                  (pi/180 * 30) * ones(1, num_arm_joints)];
    
% Optimization setup
gs = GlobalSearch; opts = optimoptions(@fmincon,'Algorithm','sqp','TolCon',1e-2);%,'FinDiffRelStep',[1e-2 * ones(1, num_arm_joints), 1e-3 * ones(1, num_arm_joints)]);
cost_func = @(x)runinv(x, num_intervals_each_joint, ...
    base_sensor_base_frame_position_base_frame, rw_params, ...
    base_rw_joint_sensor_frame_position_sensor_frame);
nonl_con = @(x)nonlinear_constraints(x, min_jt_angle, max_jt_angle, ...
                       min_jt_speed, max_jt_speed, min_jt_acc, max_jt_acc);
tic;
problem = createOptimProblem('fmincon', 'objective', cost_func, 'nonlcon',...
    nonl_con, 'x0', tr_par_seed_0, 'lb', tr_par_seed_lb, 'ub', tr_par_seed_ub, ...
    'options', opts);
% [exciting_traj_par, optimal_con_num] = run(gs, problem);
[exciting_traj_par, optimal_con_num] = fmincon(cost_func, tr_par_seed_0, ...
    [], [], [], [], tr_par_seed_lb, tr_par_seed_ub, nonl_con, opts);
time_taken = toc;
fclose(fileID);
