function [param_vec, res_norm, exit_flag] = boundary_value_least_squares(reg_mat, out_vec, lb, ub)
    gs = GlobalSearch;
    opts = optimoptions(@fmincon, 'Algorithm', 'active-set');
    problem = createOptimProblem('fmincon', 'objective', ...
        @(x)norm(reg_mat * x.' - out_vec), 'lb', lb, 'ub', ub, 'x0', ub, ...
        'options', opts);
    [param_vec, res_norm] = run(gs,problem);
end

