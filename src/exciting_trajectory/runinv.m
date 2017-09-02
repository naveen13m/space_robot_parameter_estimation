% ReDySim runinv module to perform inverse dynamics.
% Contributors of the native code: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi
% Code is modified to meet the requirements of this project

function total_cost = runinv(tr_par, base_sensor_base_frame_position_base_frame)
%   Compute system's statevar
    global iter cond_num_reg_mat inverse_signal_strength cost_value mtum_conserved;
    iter = iter + 1;
    [yo, ti, tf, incr, rtol, atol]=initials();
    options = odeset('AbsTol', atol, 'RelTol', rtol, 'stats', 'on');
    [all_instants, base_state]=ode45(@sys_ode, ti:incr:tf, yo, options, tf, tr_par);
    [num_links, not_planar] = inputs();
    is_planar = 1 - not_planar;
    num_instants = length(all_instants);
    num_joints = num_links - 1;
    joint_position = zeros(num_joints, num_instants);
    joint_velocity = zeros(num_joints, num_instants);
    
    base_com_position = base_state(:, 1 : 3);
    base_com_orientation = base_state(:, 4 : 6);
    base_com_lin_vel = base_state(:, 7 : 9);
    base_com_eul_rates = base_state(:, 10 : 12);
    for curr_instant = 1 : num_instants
        curr_time = all_instants(curr_instant);
        [joint_position(1 : num_joints, curr_instant), ...
            joint_velocity(1 : num_joints, curr_instant)] = ...
        trajectory(curr_time, num_joints + 1, tf, tr_par);
    end
    joint_position = joint_position.';
    joint_velocity = joint_velocity.';

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
    global_kin_mat(1 : 12, :) = [];
    [reg_mat, out_vec] = reduce_gkm(global_kin_mat, is_planar);
    cond_num_reg_mat = cond(reg_mat);
    sensor_signal = [base_sensor_lin_velocity,...
        base_com_ang_vel, joint_velocity];
    if is_planar
        sensor_signal(:, [3, 4, 5]) = [];
    end
    inverse_signal_strength = compute_signal_strength(sensor_signal);
    total_cost = cond_num_reg_mat + inverse_signal_strength; 
    cost_value = total_cost;
    
%     verification
    param_vec = [88.61,520,10,0,14.55,15,0,4,5,0].';
    mtum = reg_mat * param_vec([1, 3 : end]) - param_vec(2) * out_vec;
%     subplot(1, 3, 1);
%     plot(mtum(1 : 3 : end));
%     subplot(1, 3, 2);
%     plot(mtum(2 : 3 : end));
%     subplot(1, 3, 3);
%     plot(mtum(3 : 3 : end));
    mtum_conserved = 0;
    if abs(mtum) < 10e-3
        mtum_conserved = 1;
    end
end