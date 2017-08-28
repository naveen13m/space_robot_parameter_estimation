clear all; close all; clc

% dbstop in runinv.m at 63
% Config params
global A_eq b_eq
num_links = inputs();
num_harmonics = 3;
[min_jt_angle, max_jt_angle, min_jt_speed, max_jt_speed, min_jt_acc, ...
    max_jt_acc] = joint_limits();
base_sensor_base_frame_position_base_frame = [0; 0; 0];

gs = GlobalSearch; opts = optimoptions(@fmincon,'Algorithm','sqp');
num_joints = num_links - 1;
num_traj_params = num_harmonics * 2 + 2;
tr_par_0 = [0 0.5  0 0 0  1 0 0, 0 0.5  0 0 0 -1.5 0 0];

A_eq = [0 0, 1 2 3, 0 0 0, zeros(1, 8);
        zeros(1, 8), 0 0, 1 2 3, 0 0 0];
    
b_eq = [0;
        0];    

nonl_con = @(x)nonlinear_constraints(x, min_jt_angle, max_jt_angle, min_jt_speed, ...
    max_jt_angle, min_jt_acc, max_jt_acc);
cost_func = @(x)runinv(x, base_sensor_base_frame_position_base_frame);
problem = createOptimProblem('fmincon', 'objective', cost_func, 'nonlcon',...
    nonl_con, 'x0', tr_par_0, 'Aeq', A_eq, 'beq', b_eq, 'options', opts);
[exciting_traj_par, optimal_con_num] = run(gs,problem);
