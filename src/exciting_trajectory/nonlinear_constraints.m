function [c, ceq] = nonlinear_constraints(tr_par_seed, th_l, th_u, dth_l, dth_u, ddth_l, ddth_u)
    global fileID iter cond_num_reg_mat inverse_signal_strength cost_value mtum_conserved
    num_links = inputs();
    num_arm_joints = num_links - 1 - 4;
%     [~, ti, tf, incr, ~, ~, vel_combi_mat] = initials();
%     num_intervals_each_joint = size(vel_combi_mat, 1);
%     all_instants = ti : incr : tf;
%     num_instants = length(all_instants);
%     th = zeros(num_joints, num_instants); 
%     tr_par = make_tr_params(tr_par_seed, vel_combi_mat);
%     dth = zeros(num_joints, num_instants); 
%     ddth = zeros(num_joints, num_instants);
%     for curr_instant_index = 1 : num_instants 
%         [th(:, curr_instant_index), dth(:, curr_instant_index), ddth(:, curr_instant_index)] = ...
%             trajectory(all_instants(curr_instant_index), num_links, tf, tr_par, num_intervals_each_joint);
%     end
    load joint_data.mat;
    th = joint_position(:, 1 : num_arm_joints).';
    dth = joint_velocity(:, 1 : num_arm_joints).';
    ddth = joint_acc(:, 1 : num_arm_joints).';
    delete joint_data.mat
    
    th_comp_min = min(th,[],2);
    th_comp_max = max(th,[],2);
    thd_comp_min = min(dth,[],2);
    thd_comp_max = max(dth,[],2);
    thdd_comp_min = min(ddth,[],2);
    thdd_comp_max = max(ddth,[],2);
    
    % Inequality constraints (c <= 0)
    % Checking if the pos, vel and acc traj are within the limits (c <= 0)
    c = [th_l - th_comp_min;
         th_comp_max - th_u;
         dth_l - thd_comp_min;
         thd_comp_max - dth_u;
         ddth_l - thdd_comp_min;
         thdd_comp_max - ddth_u];
        
    % Equality constraints (ceq = 0) 
    ceq = [];
    
    
    % Store data into textfile for post-opti analysis
    % Iter index
    fprintf(fileID, 'Iter_index = %d\n', iter);
    
    % Traj params
    lnt = length(tr_par_seed);
    fprintf(fileID, 'tr_par_seed = [');
    for i = 1 : lnt
        fprintf(fileID, '%f, ', tr_par_seed(i));
    end
    fprintf(fileID, ']\n');
    
    % Mtum conservation verification
    fprintf(fileID, 'Mtum_conserved = %d\n', mtum_conserved);
    
      
    % Nonlinear inequality
    if c < 10e-3
        fprintf(fileID, 'Nonlinear inequality satisfied = Yes\n');  
    else
        fprintf(fileID, 'Nonlinear inequality satisfied = No\n');  
    end
        
    lnt = length(c);
    fprintf(fileID, 'c = [');
    for i = 1 : lnt
        fprintf(fileID, '%f, ', c(i));
    end
    fprintf(fileID, ']\n');
    
    fprintf(fileID, 'Con_no = %f, inverse_signal_strength = %f, total_cost = %f\n', ...
        cond_num_reg_mat(iter), inverse_signal_strength(iter), cost_value(iter));
    fprintf(fileID, '-------------------------------------------------------------\n');
end