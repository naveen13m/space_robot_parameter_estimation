clear all; close all; clc

% dbstop in compute_signal_strength.m at 22 
% Results verification
global iter fileID A_ineq b_ineq
iter = 0;
filename = 'opti_data.txt';
fileID = fopen(filename, 'w');

% System configuration
num_links = inputs();
[min_jt_angle, max_jt_angle, min_jt_speed, max_jt_speed, min_jt_acc, max_jt_acc] = joint_limits();
base_sensor_base_frame_position_base_frame = [0; 0; 0];
[~, ~, tf] = initials();

% Upper and lower bounds on the trajectory coeffs
num_joints = num_links - 1;
num_intervals_each_joint = 2 ^ (num_joints - 1);
num_coeffs_each_joint = num_intervals_each_joint + 1;
for curr_joint = 1 : num_joints
    start_index = (curr_joint - 1) * num_coeffs_each_joint + 1;
    end_index = curr_joint * num_coeffs_each_joint;
    tr_par_lb(1, start_index : end_index) = repmat(min_jt_angle(curr_joint), 1, num_coeffs_each_joint);
    tr_par_ub(1, start_index : end_index) = repmat(max_jt_angle(curr_joint), 1, num_coeffs_each_joint);
end
% Initial guess of trajectory coeffs
tr_par_0  = [-0.0346803245105745,1.57079632679490,1.57079632679490,-0.0872664625997165,2.09439510239320,-0.0872664625997166];

% Inequality constraints for the coeffs to comply with vel. combi.
vel_combi_mat = [1 1 
                 1 0];
A_ineq = zeros(num_joints * num_intervals_each_joint, num_joints * num_coeffs_each_joint);             
for curr_joint = 1 : num_joints
    num_coeffs_till_curr_joint = num_coeffs_each_joint * (curr_joint - 1);
    num_intervals_till_curr_joint = num_intervals_each_joint * (curr_joint - 1);
    for curr_interval = 1 : num_intervals_each_joint
        thi_index = num_coeffs_till_curr_joint + curr_interval;
        thf_index = num_coeffs_till_curr_joint + curr_interval + 1;
        row_index = num_intervals_till_curr_joint + curr_interval;
        if vel_combi_mat(curr_interval, curr_joint) == 1
            A_ineq(row_index, thi_index) = 1;
            A_ineq(row_index, thf_index) = -1;
        else
            A_ineq(row_index, thi_index) = -1;
            A_ineq(row_index, thf_index) = 1;
        end
    end
end
b_ineq = zeros(num_joints * num_intervals_each_joint, 1);
    
% Global optimization routine
gs = GlobalSearch; opts = optimoptions(@fmincon,'Algorithm','sqp','TolCon', 10e-3);
nonl_con = @(x)nonlinear_constraints(x, min_jt_speed, max_jt_speed, min_jt_acc, max_jt_acc);
cost_func = @(x)runinv(x, base_sensor_base_frame_position_base_frame);
problem = createOptimProblem('fmincon', 'objective', cost_func, 'nonlcon',...
    nonl_con, 'x0', tr_par_0, 'lb', tr_par_lb, 'ub', tr_par_ub, ...
    'Aineq', A_ineq, 'bineq', b_ineq, 'options', opts);
tic;
[exciting_traj_par, optimal_con_num] = run(gs, problem);
time_taken = toc;
fclose(fileID);
