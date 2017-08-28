function global_kin_mat = eliminate_dependency(global_kin_mat)
    global_kin_mat(1 : 6, :) = [];
    global_kin_mat([3 : 6 : end, 4 : 6 : end, 5 : 6 : end], :) = [];
    global_kin_mat(:, [1 : 10 : end, 2 : 10 : end, 4 : 10 : end, 5 : 10 : end, ...
        6 : 10 : end, 10 : 10 : end, 17 : 10 : end]) = [];
end