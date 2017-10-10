% ReDySim runinv module to perform inverse dynamics.
% Contributors of the native code: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi
% Code is modified to meet the requirements of this project

function total_cost = runinv(tr_par_seed, num_intervals_each_joint,...
                        base_sensor_base_frame_position_base_frame, rw_params)
%   Compute system's statevar
    global iter cond_num_reg_mat inverse_signal_strength cost_value mtum_conserved;
    iter = iter + 1;
    [yo, ti, tf, incr, rtol, atol, vel_combi_mat]=initials();
    options = odeset('AbsTol', atol, 'RelTol', rtol, 'stats', 'on');
    tr_par = make_tr_params(tr_par_seed, vel_combi_mat);
    [all_instants, base_state]=ode45(@sys_ode, ti:incr:tf, yo, options,...
                                    tf, tr_par, num_intervals_each_joint);
    [num_links, not_planar] = inputs();
    is_planar = 1 - not_planar;
    num_instants = length(all_instants);
    num_joints = num_links - 1;
    joint_position = zeros(num_joints, num_instants);
    joint_velocity = zeros(num_joints, num_instants);
    joint_acc = zeros(num_joints, num_instants);
    
    base_com_position = base_state(:, 1 : 3);
    base_com_orientation = base_state(:, 4 : 6);
    base_com_lin_vel = base_state(:, 7 : 9);
    base_com_eul_rates = base_state(:, 10 : 12);
    for curr_instant = 1 : num_instants
        curr_time = all_instants(curr_instant);
        [joint_position(1 : num_joints, curr_instant), ...
            joint_velocity(1 : num_joints, curr_instant), ...
            joint_acc(1 : num_joints, curr_instant)] = ...
        trajectory(curr_time, num_joints + 1, tf, tr_par, num_intervals_each_joint);
    end
    joint_position = joint_position.';
    joint_velocity = joint_velocity.';
    joint_acc = joint_acc.';
    save joint_data.mat joint_position joint_velocity joint_acc

    % Convert COM data to sensor frame data and euler rates to body rates
    base_sensor_position = zeros(num_instants, 3);
    base_sensor_lin_velocity = zeros(num_instants, 3);
    base_com_ang_vel = zeros(num_instants, 3);
    
    for curr_instant = 1 : num_instants
        phi = base_com_orientation(curr_instant, 1);
        theta = base_com_orientation(curr_instant, 2);
        psi = base_com_orientation(curr_instant, 3);
        rot_mat_base = ZXY_euler_to_rot_mat(phi, theta, psi);
        ZXY_euler_rates_to_body_rates = [-sin(psi)*cos(theta)   cos(psi)  0
                                          sin(theta)            0         1
                                          cos(psi)*cos(theta)   sin(psi)  0];
        base_com_ang_vel(curr_instant, :) = rot_mat_base * ...
            ZXY_euler_rates_to_body_rates * base_com_eul_rates(curr_instant, :).';
        base_sensor_base_com_position = rot_mat_base * ...
            base_sensor_base_frame_position_base_frame;
        base_sensor_position(curr_instant, :) = ...
        (base_com_position(curr_instant, 1:3).' + ...
            base_sensor_base_com_position).' - ...
            base_sensor_base_frame_position_base_frame.';
        base_sensor_lin_velocity(curr_instant, :) = ...
        (base_com_lin_vel(curr_instant, :).' + ...
            cross(base_com_ang_vel(curr_instant, :).', ...
            base_sensor_base_com_position)).';
    end
    
    statevar = [base_sensor_position, base_com_orientation, joint_position, ...
        base_sensor_lin_velocity, base_com_ang_vel, joint_velocity];
    
    global_kin_mat = global_kinematic_matrix_modified...
        (base_sensor_base_frame_position_base_frame, statevar);
%     global_kin_mat(1 : 12, :) = [];
    [reg_mat, out_vec] = reduce_gkm(global_kin_mat, is_planar, rw_params);
    cond_num_reg_mat(iter) = cond(reg_mat);
    sensor_signal = [base_sensor_lin_velocity, base_com_ang_vel, joint_velocity(:, 1 : end - 4)];
    if is_planar
        sensor_signal(:, [3, 4, 5]) = [];
    end
    inverse_signal_strength(iter) = compute_signal_strength(sensor_signal);
    total_cost = cond_num_reg_mat(iter) + 0.2 * inverse_signal_strength(iter); 
    cost_value(iter) = total_cost;
    
% %     verification
% %     Dual arm articulate
%     param_vec = [2064.06000000000,2012.06000000000,1549.40000000000,-99.7800000000000,-260,-158.340000000000,2230,451,669,1001.50000000000,-3.44500000000002,192.775000000000,7.40000000000000,1.82500000000001,4.60000000000000,100,-4.50000000000000,-36.8660000000000,55.1340000000000,2.25000000000000,1.67000000000000,-6.70000000000000,46,-1.60000000000000,-14.0100000000000,55.0300000000000,-1.62000000000000,5.50000000000000,3.93000000000000,21,12,-36.2250000000000,130.845000000000,0.900000000000000,1.43000000000000,4.31000000000000,87.5000000000000,1.50000000000000,-32.2900000000000,46.4550000000000,0.442500000000000,1.46250000000000,1.73250000000000,40.7500000000000,1.75000000000000,-3.21000000000000,13.8950000000000,1.53750000000000,1.45750000000000,2.10750000000000,8.75000000000000,-1.25000000000000].';
% %     3-link planar
%     param_vec = [676.4,1080,256,324,62.5,55,0,17.5,15,0].';
%     comp_param = pinv(reg_mat) * out_vec;
%     compare = [param_vec, comp_param]
%     mtum = reg_mat * param_vec - out_vec;
%     
%     if is_planar
%         subplot(1, 3, 1);
%         plot(mtum(1 : 6 : end));
%         subplot(1, 3, 2);
%         plot(mtum(2 : 6 : end));
%         subplot(1, 3, 3);
%         plot(mtum(3 : 6 : end));
%         drawnow;
%     else
%         subplot(2, 3, 1);
%         plot(mtum(1 : 6 : end));
%         subplot(2, 3, 2);
%         plot(mtum(2 : 6 : end));
%         subplot(2, 3, 3);
%         plot(mtum(3 : 6 : end));
%         subplot(2, 3, 4);
%         plot(mtum(4 : 6 : end));
%         subplot(2, 3, 5);
%         plot(mtum(5 : 6 : end));
%         subplot(2, 3, 6);
%         plot(mtum(6 : 6 : end));
%         drawnow;
%     end
    
    mtum_conserved = 1;
%     if abs(mtum) < 10e-3
%         mtum_conserved = 1;
%     end
end