function [c, ceq] = nonlinear_constraints(tr_par, min_vel, max_vel, min_acc, max_acc)
    global fileID iter cond_num_reg_mat inverse_signal_strength cost_value mtum_conserved A_ineq b_ineq;
    num_links = inputs();
    num_joints = num_links - 1;
    [~, ~, tf, ~, ~, ~] = initials();
    
    % Compute min and max joint velocity
    dth_comp_min = [];
    dth_comp_max = [];
    ddth_comp_min = [];
    ddth_comp_max = [];
    min_interval_vel = [];
    max_interval_vel = [];
    min_interval_acc = [];
    max_interval_acc = [];
    time_params = [];
        
    for curr_joint = 1 : num_joints
        num_intervals_each_joint = 2 ^ (num_joints - 1);
        num_curr_joint_coeff_params = num_intervals_each_joint + 1;
        num_terms_till_curr_joint = num_curr_joint_coeff_params * (curr_joint - 1);
        for curr_interval = 1 : num_intervals_each_joint
            thi = tr_par(num_terms_till_curr_joint + curr_interval);
            thf = tr_par(num_terms_till_curr_joint + curr_interval + 1);
            Tp = tf / num_intervals_each_joint;
            time_params = [time_params; Tp];
            if thi < thf
                curr_interval_min_vel = 0;
                curr_interval_max_vel = (2/Tp) * (thf - thi);
                curr_interval_min_acc = (2 * pi/Tp^2) * (thi - thf);
                curr_interval_max_acc = (2 * pi/Tp^2) * (thf - thi);
            else
                curr_interval_min_vel = (2/Tp) * (thf - thi);
                curr_interval_max_vel = 0;
                curr_interval_min_acc = (2 * pi/Tp^2) * (thf - thi);
                curr_interval_max_acc = (2 * pi/Tp^2) * (thi - thf);
            end
            dth_comp_min = [dth_comp_min; curr_interval_min_vel];
            dth_comp_max = [dth_comp_max; curr_interval_max_vel];
            ddth_comp_min = [ddth_comp_min; curr_interval_min_acc];
            ddth_comp_max = [ddth_comp_max; curr_interval_max_acc];            
        end
        min_interval_vel = [min_interval_vel; repmat(min_vel(curr_joint, 1), num_intervals_each_joint, 1)];
        max_interval_vel = [max_interval_vel; repmat(max_vel(curr_joint, 1), num_intervals_each_joint, 1)];
        min_interval_acc = [min_interval_acc; repmat(min_acc(curr_joint, 1), num_intervals_each_joint, 1)];
        max_interval_acc = [max_interval_acc; repmat(max_acc(curr_joint, 1), num_intervals_each_joint, 1)];    
    end          
    
    % Inequality constraints (c <= 0)
    % Checking if the vel and acc traj are within the limits (c <= 0)
    c = [min_interval_vel - dth_comp_min;
         dth_comp_max - max_interval_vel;
         min_interval_acc - ddth_comp_min;
         ddth_comp_max - max_interval_acc];
        
    % Equality constraints (ceq = 0) 
    ceq = [];
    
    
    % Store data into textfile for post-opti analysis
    % Iter index
    fprintf(fileID, 'Iter_index = %d\n', iter);
    
    % Traj params
    lnt = length(tr_par);
    fprintf(fileID, 'tr_par = [');
    for i = 1 : lnt
        fprintf(fileID, '%f, ', tr_par(i));
    end
    fprintf(fileID, ']\n');
    
    % Mtum conservation verification
    fprintf(fileID, 'Mtum_conserved = %d\n', mtum_conserved);
    
    % Linear inequality
    c_lin = A_ineq * tr_par.' - b_ineq;
    if c_lin < 10e-3
        fprintf(fileID, 'Linear inequality satisfied = Yes\n');  
    else
        fprintf(fileID, 'Linear inequality satisfied = No\n');  
    end
    lnt = length(c_lin);
    fprintf(fileID, 'c_lin = [');
    for i = 1 : lnt
        fprintf(fileID, '%f, ', c_lin(i));
    end
    fprintf(fileID, ']\n');
    
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
        cond_num_reg_mat, inverse_signal_strength, cost_value);
    fprintf(fileID, '-------------------------------------------------------------\n');
end