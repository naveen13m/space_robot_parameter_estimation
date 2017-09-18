clear all; close all; clc

% dbstop in compute_signal_strength.m at 22 
% dbstop in runinv.m at 101
% Results verification
global iter fileID cost_value cond_num_reg_mat inverse_signal_strength
iter = 0;
filename = 'opti_data.txt';
fileID = fopen(filename, 'w');

% Config params
% % System configuration
num_links = inputs();
[min_jt_angle, max_jt_angle, min_jt_speed, max_jt_speed, min_jt_acc,...
                                              max_jt_acc] = joint_limits();
base_sensor_base_frame_position_base_frame = [-0.2; -0.3; -0.4];
[~, ~, tf, ~, ~, ~, vel_combi_mat] = initials();
rw_params = make_reaction_wheel();
num_rw_joints = 4;              
num_arm_joints = num_links - 1 - num_rw_joints;
num_intervals_each_joint = size(vel_combi_mat, 1);
num_coeffs_each_joint = num_intervals_each_joint + 1;



% % Opti Config
% % % Initial guess, lower and upper bounds of trajectory seed coeffs
tr_par_seed_0 = [-pi/2, pi/6, -pi/4, pi, -pi/6, 0, pi/180 * 2.5, pi/180 * 9, pi/180 * 9, pi/180 * 5, pi/180 * 9, pi/180 * 9];
tr_par_seed_lb = [min_jt_angle.', (pi/180 * 0.5) * ones(1, num_arm_joints)];
tr_par_seed_ub = [max_jt_angle.', (pi/180 * 30) * ones(1, num_arm_joints)];
    
% Optimization setup
gs = GlobalSearch; opts = optimoptions(@fmincon,'Algorithm','sqp','TolCon',1e-2,'FinDiffRelStep',1e-2);
nonl_con = @(x)nonlinear_constraints(x, min_jt_angle, max_jt_angle, ...
    min_jt_speed, max_jt_speed, min_jt_acc, max_jt_acc);
cost_func = @(x)runinv(x, num_intervals_each_joint, ...
    base_sensor_base_frame_position_base_frame, rw_params);
tic;
% problem = createOptimProblem('fmincon', 'objective', cost_func, 'nonlcon',...
%     nonl_con, 'x0', tr_par_seed_0, 'lb', tr_par_seed_lb, 'ub', tr_par_seed_ub, ...
%     'options', opts);
% [exciting_traj_par, optimal_con_num] = run(gs, problem);
[exciting_traj_par, optimal_con_num] = fmincon(cost_func, tr_par_seed_0, ...
    [], [], [], [], tr_par_seed_lb, tr_par_seed_ub, nonl_con, opts);
time_taken = toc;
fclose(fileID);
