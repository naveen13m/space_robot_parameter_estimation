clear all; 
close all;
clc;

% global to_check
% dbstop in global_kinematic_matrix.m  at 196
% Configuration parameters
% robot_make = '/1_link_rw';
% robot_make = '/1_link_2rw';
% robot_make = '/2_link';
% robot_make = '/2_link_mrw';
robot_make = '/3_link';
% robot_make = '/4_link';
% robot_make = '/8_link';
% robot_make = '/dual_arm';
% robot_make = '/2_link_rw';
% robot_make = '/4_link_spatial';
% robot_make = '/temp';
% robot_make = '/static_base';

base_sensor_position_base_frame = [0; 0; 0];
% base_sensor_position_base_frame = [-0.2; -0.3; 0];
% base_sensor_position_base_frame = [0.1; 0.2; 0];
sim_data = 1;
data_dir = {'/experimental_data', '/sim_real_data'};
experimental_data_filename = {'/statevar.dat', '/timevar.dat'};
real_data_filename = {'/statevar.dat', '/timevar.dat', '/mtvar.dat'};
data_filename = [{experimental_data_filename}, {real_data_filename}];

% Load data to workspace
curr_dir = pwd;
data_filename = data_filename{sim_data  + 1};
num_data_files = length(data_filename);
for i = 1 : num_data_files
    load(strcat(curr_dir(1 : end - 4), '/test_case_data', ...
        robot_make, data_dir{sim_data + 1}, data_filename{i}));
end

% Load robot structural and dynamic parameter data
curr_dir = pwd;
cd(strcat('../test_case_data', robot_make, '/config_files'));
[num_links_with_base, not_planar, joint_twist_angle, link_length_DH, ~, parent_link_index, ...
    link_x_com, link_y_com, link_z_com, link_length, ~, link_mass] = ...
    inputs();
cd(curr_dir);
num_instants = size(statevar, 1);

% Populate the linear momentum regressor matrix
global_kin_mat = global_kinematic_matrix(robot_make, ...
    base_sensor_position_base_frame, statevar, mtvar);
reg_mat = remove_excess(global_kin_mat);
rref_reg_mat = rref(reg_mat, 10e-5);
[rref_global_kin_mat, base_col_index] = rref(global_kin_mat);

reg_mat = global_kin_mat(:, [3, 7, 8, 9, 13, 18, 19, 23, 28, 29]);
reg_mat([1 : 6, 9 : 6 : end, 10 : 6 : end, 11 : 6 : end]) = [];
reg_mat = global_kin_mat(:, [3, 7, 8, 9, 13, 18, 19, 23, 28, 29]);
reg_mat([1 : 6, 9 : 6 : end, 10 : 6 : end, 11 : 6 : end], :) = [];
b = - 1080 * reg_mat(:, 2);
A = reg_mat(:, [1, 3 : end]);

% % disp('Simple LS')
% ls_sol = pinv(A) * b

%    --    Base   ---  --  Link-1 --  --  Link-2   --
lb = [400, -100, -100,    0,   0,  0,    15,   0,  0];
ub = [600,  100,  100,  100, 100,  2,    30,  20,  5];
act_sol = [520,   40,    0, 62.5,  55,  0,  17.5,  15,  0];

% % Verification
mtum_act = A*act_sol.' - b;
figure(1);
plot(mtum_act(1 : 3 : end)); hold on;
figure(2);
plot(mtum_act(2 : 3 : end)); hold on;
figure(3);
plot(mtum_act(3 : 3 : end)); hold on;

% % Constrained LS
disp('Constrained LS')
[cls_sol, resnorm, ~, exitflag, output, lambda] = lsqlin(A, b, [], [], [], [], lb, ub, ub)

% % Particle filter
% options = optimoptions('particleswarm','SwarmSize',200,'HybridFcn',@patternsearch);
% [pf_sol, fval, exitflag, output] = particleswarm(@(x)(norm((A * x.' - b))), 9, lb, ub, options)

% % Global search
disp('Global search')
gs = GlobalSearch;
opts = optimoptions(@fmincon,'Algorithm','sqp');
problem = createOptimProblem('fmincon','objective',@(x)norm(A * x.' - b),'lb',lb,'ub',ub,'x0',cls_sol.', 'options', opts);
[gs_sol, res_norm] = run(gs,problem)

% Verification
mtum_comp = A*gs_sol.' - b;
figure(1);
plot(mtum_comp(1 : 3 : end)); legend('P_{xa}','P_{xx}');
figure(2);
plot(mtum_comp(2 : 3 : end)); legend('P_{ya}', 'P_{yc}');
figure(3);
plot(mtum_comp(3 : 3 : end)); legend('L_{za}', 'L_{zc}');