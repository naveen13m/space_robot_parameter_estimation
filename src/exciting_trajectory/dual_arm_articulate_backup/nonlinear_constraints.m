function [c, ceq] = nonlinear_constraints(tr_par_seed, th_l, th_u, dth_l, dth_u, ddth_l, ddth_u)
    global fileID iter cond_num_reg_mat inverse_signal_strength cost_value mtum_conserved
    [num_links, not_planar] = inputs();
    is_planar = 1 - not_planar;
    if is_planar
        num_rw_joints = 1;       
    else
        num_rw_joints = 4;
    end
    num_arm_joints = num_links - 1 - num_rw_joints;
    
    load joint_data.mat;
    th = joint_position(:, 1 : num_arm_joints).';
    dth = joint_velocity(:, 1 : num_arm_joints).';
    ddth = joint_acc(:, 1 : num_arm_joints).';
    
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
    if c < 10e-2
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