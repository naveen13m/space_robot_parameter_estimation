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
% robot_make = '/3_link';
% robot_make = '/3_link_rw';
% robot_make = '/4_link';
robot_make = '/4_link_spatial';
% robot_make = '/8_link';
% robot_make = '/dual_arm';
% robot_make = '/2_link_rw';
% robot_make = '/temp';
% robot_make = '/static_base';

base_sensor_base_frame_position_base_frame = [0; 0; 0];
% base_sensor_base_frame_position_base_frame = [-0.2; -0.3; 0];
% base_sensor_base_frame_position_base_frame = [0.1; 0.2; 0];
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
cd(strcat('../test_case_data', robot_make, '/config_files'));
[num_links, not_planar, joint_twist_angle, link_length_DH, ...
    joint_offset, parent_link_index, link_x_com, link_y_com, link_z_com, ...
    ~, ~, link_mass, ~, inertia_xx_com, inertia_yy_com, inertia_zz_com, ...
    inertia_xy_com, inertia_yz_com, inertia_zx_com] = inputs();
link_x_com(1) = -base_sensor_base_frame_position_base_frame(1);
link_y_com(1) = -base_sensor_base_frame_position_base_frame(2);
link_z_com(1) = -base_sensor_base_frame_position_base_frame(3);
is_planar = 1 - not_planar;
cd(curr_dir);
param_vector = construct_dyn_param_vector(...
        inertia_xx_com, inertia_yy_com, inertia_zz_com, inertia_xy_com, inertia_yz_com,...
        inertia_zx_com, link_mass, link_x_com, link_y_com, link_z_com, num_links);
num_instants = size(statevar, 1);

% Populate the linear momentum regressor matrix
global_kin_mat = global_kinematic_matrix(robot_make, ...
    base_sensor_base_frame_position_base_frame, statevar, mtvar);
[rref_global_kin_mat, base_col_index] = rref(global_kin_mat);

% Remove unnecessary stuff
if is_planar
    reg_mat = remove_excess(global_kin_mat);
%     reg_mat(271 : end, : ) = [];
else
    reg_mat = global_kin_mat;
end

A = reg_mat(:, [1, 3 : end]);
cn_before_precon = cond(A);

% Preconditioning
precon = precondition;
reg_mat = precon.scaling(reg_mat);

% Segregating coeff mat and data vec
b = -sum(link_mass) * reg_mat(:, 2);
A = reg_mat(:, [1, 3 : end]);
cn_after_precon = cond(A);

% % Simple LS
% disp('Simple LS')
ls_sol = pinv(A) * b;
fake_res_norm = norm(A*ls_sol - b);

%    --    Base   ---  --  Link-1 --  --  Link-2   --
lb = [400, -100, -100,    0,   0,  0,    10,   0,  0];
ub = [600,  100,  100,  100, 100,  2,    30,  20,  5];
act_sol = [520,   40,    0, 62.5,  55,  0,  17.5,  15,  0];
act_res_norm = norm(A * act_sol.' - b);

% % BVLS
% disp('BVLS')
[BVLS_sol, resnorm, ~, exitflag, output] = lsqlin(A, b, [], [], [], [], lb, ub, ub)
gs = GlobalSearch;
opts = optimoptions(@fmincon,'Algorithm','sqp');
problem = createOptimProblem('fmincon','objective',@(x)norm(A * x.' - b),'lb',lb,'ub',ub,'x0', BVLS_sol.', 'options', opts);
[gs_sol, res_norm] = run(gs,problem)


% Verification
mtum_act = A*act_sol.' - b;
mtum_comp = A*gs_sol.' - b;
figure();
subplot(1, 3, 1)
plot(mtum_act(1 : 3 : end)); hold on;
plot(mtum_comp(1 : 3 : end)); legend('P_{xa}','P_{xc}');
subplot(1, 3, 2)
plot(mtum_act(2 : 3 : end)); hold on;
plot(mtum_comp(2 : 3 : end)); legend('P_{ya}', 'P_{yc}');
subplot(1, 3, 3)
plot(mtum_act(3 : 3 : end)); hold on;
plot(mtum_comp(3 : 3 : end)); legend('L_{za}', 'L_{zc}');