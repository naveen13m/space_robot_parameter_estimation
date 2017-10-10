function sol = unnamed_method(reg_mat, out_vec, x0, split_end_index)
    max_iter = 2000;
    conv_tol = 1e-6;
    num_splits = length(split_end_index);
    split_start_index = [1, split_end_index(1 : end - 1) + 1];
    sol = x0;
    curr_iter = 0;
    curr_tol = 100;
    while curr_iter < max_iter && curr_tol > conv_tol
        prev_sol = sol;
        curr_iter = curr_iter + 1;
        for curr_split_index = 1 : num_splits
            curr_split_start_index = split_start_index(curr_split_index);
            curr_split_end_index = split_end_index(curr_split_index);
            curr_split_reg_mat = reg_mat(:, curr_split_start_index : curr_split_end_index);
            curr_split_out_vec = compute_split_out_vec(reg_mat, out_vec, sol, ...
                        split_start_index, split_end_index, num_splits, curr_split_index);
            sol(curr_split_start_index : curr_split_end_index) = ...
                pinv(curr_split_reg_mat) * curr_split_out_vec;
        end
        curr_tol = norm(sol - prev_sol);
        sol
    end
end

function curr_split_out_vec = compute_split_out_vec(reg_mat, out_vec, sol, ...
                        split_start_index, split_end_index, num_splits, ignore_split_index)
    curr_split_out_vec = out_vec;
    for curr_split_index = 1 : num_splits
        if curr_split_index ~= ignore_split_index
            curr_split_start_index = split_start_index(curr_split_index);
            curr_split_end_index = split_end_index(curr_split_index);
            curr_split_reg_mat = reg_mat(:, [curr_split_start_index : curr_split_end_index]);
            curr_split_sol = sol(curr_split_start_index : curr_split_end_index);
            curr_split_out_vec = curr_split_out_vec - curr_split_reg_mat * curr_split_sol;
        end
    end
end                                  