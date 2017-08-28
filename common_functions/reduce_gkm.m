function [reg_mat, output_vec] = reduce_gkm(gkm, is_planar)
    if is_planar
        reduced_mat = eliminate_dependency(gkm);
    else
        reduced_mat = gkm;
        reduced_mat(:, [12 : 10 : end, 17 : 19 : end, 20 : 10 : end]) = [];
    end
    reg_mat = reduced_mat(:, [1, 3 : end]);
    output_vec = -reduced_mat(:, 2);
end

