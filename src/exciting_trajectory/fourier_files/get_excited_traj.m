clear all; close all; clc

dbstop in runinv.m at 85
global iter fileID
filename = 'opti_data.txt';
fileID = fopen(filename, 'w');

% Config params
num_links = inputs();
num_harmonics = 3;
[min_jt_angle, max_jt_angle, min_jt_speed, max_jt_speed, min_jt_acc, ...
    max_jt_acc] = joint_limits();
base_sensor_base_frame_position_base_frame = [0; 0; 0];
iter = 0;

gs = GlobalSearch; opts = optimoptions(@fmincon,'Algorithm','sqp', 'TolCon', 10e-3);
num_joints = num_links - 1;
num_traj_params = num_harmonics * 2 + 2;

tr_par_lb = [-pi pi/20, -pi/2 -pi/2 -pi/2 -pi/2 -pi/2,  -pi/2 -pi/2 -pi/2 -pi/2 -pi/2, -pi pi/10, -pi/2 -pi/2 -pi/2 -pi/2 -pi/2, -pi/2 -pi/2 -pi/2 -pi/2 -pi/2]; 
tr_par_ub = [ pi pi/20,  pi/2  pi/2  pi/2  pi/2  pi/2,   pi/2  pi/2  pi/2  pi/2  pi/2,  pi pi/10,  pi/2  pi/2  pi/2  pi/2  pi/2,  pi/2  pi/2  pi/2  pi/2  pi/2]; 
% tr_par_0 = [0 pi/10  0 0 0  0.05 -0.15 0, 0 pi/10  0 0 0 -0.15 0.25 0];
tr_par_0 = [0.793970359995414, pi/20,-0.120060859826561,-0.0448800825125781,0.0699403416172392, 0, 0,-0.618669486946362,-0.0475715609808641,-0.198905088483570, 0, 0, 0.666743433274697,0.314159265358979,0.232768048844532,-0.0337918047971565,-0.0550614797500729, 0, 0, -1.12608938459223,0.0917205548028718,-0.0302566873595932, 0, 0];

nonl_con = @(x)nonlinear_constraints(x, min_jt_angle, max_jt_angle, min_jt_speed, ...
    max_jt_angle, min_jt_acc, max_jt_acc);
cost_func = @(x)runinv(x, base_sensor_base_frame_position_base_frame);
problem = createOptimProblem('fmincon', 'objective', cost_func, 'nonlcon',...
    nonl_con, 'x0', tr_par_0, 'lb', tr_par_lb, 'ub', tr_par_ub, 'options', opts);
[exciting_traj_par, optimal_con_num] = run(gs,problem);
fclose(fileID);
