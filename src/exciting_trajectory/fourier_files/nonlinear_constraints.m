function [c, ceq] = nonlinear_constraints(tr_par, th_l, th_u, dth_l, dth_u, ddth_l, ddth_u)
    global fileID iter cond_num_reg_mat inverse_signal_strength cost_value mtum_conserved;
    num_links = inputs();
    num_joints = num_links - 1;
    [~, ti, tf, incr, ~, ~] = initials();
    all_instants = ti : incr : tf;
    num_instants = length(all_instants);
    th = zeros(num_joints, num_instants); 
    dth = zeros(num_joints, num_instants); 
    ddth = zeros(num_joints, num_instants);
    for curr_instant_index = 1 : num_instants 
        [th(:, curr_instant_index), dth(:, curr_instant_index), ddth(:, curr_instant_index)] = ...
            trajectory(all_instants(curr_instant_index), num_links, tf, tr_par);
    end

    % Max & min pos, vel and acc with given fourier parameters
    th_comp_min = min(th,[],2);
    th_comp_max = max(th,[],2);
    thd_comp_min = min(dth,[],2);
    thd_comp_max = max(dth,[],2);
    thdd_comp_min = min(ddth,[],2);
    thdd_comp_max = max(ddth,[],2);

    % Inequality constraints (c <= 0)
    % Checking if the position, vel and acc traj are within the limits (c <= 0)
    c = [th_l - th_comp_min;
         th_comp_max - th_u;
         dth_l - thd_comp_min;
         thd_comp_max - dth_u;
         ddth_l - thdd_comp_min;
         thdd_comp_max - ddth_u];
        
    % Equality constraints (ceq = 0) 
    ceq = [];
    
    
    % Store data into textfile for post-opti analysis
    fprintf(fileID, 'Iter_index = %d\n', iter);
    
    lnt = length(tr_par);
    fprintf(fileID, 'tr_par = [');
    for i = 1 : lnt
        fprintf(fileID, '%f, ', tr_par(i));
    end
    fprintf(fileID, ']\n');
    
    fprintf(fileID, 'Mtum_conserved = %d\n', mtum_conserved);
    
    if c < 10e-3
        fprintf(fileID, 'Nonlinear satisfied = Yes\n');  
    else
        fprintf(fileID, 'Nonlinear satisfied = No\n');  
    end
    lnt = length(c);
    fprintf(fileID, 'c = [');
    for i = 1 : lnt
        fprintf(fileID, '%f, ', c(i));
    end
    fprintf(fileID, ']\n');
    
    fprintf(fileID, 'Con_no = %f, inverse_signal_strength = %f, total_cost = %f\n', ...
        cond_num_reg_mat, inverse_signal_strength, cost_value);
    fprintf(fileID, '-------------------------------------------------------------\n');
end