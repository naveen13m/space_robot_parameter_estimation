function [reg_mat, output_vec] = reduce_gkm(gkm, is_planar)
    if is_planar
        reg_mat = eliminate_dependency(gkm);
        output_vec = -reg_mat(:, 2);
        reg_mat(:, 2) = [];
    else
        reg_mat = gkm;
        output_vec = -reg_mat(:, 7);
        reg_mat(:, [7, 12 : 10 : end, 17 : 10 : end, 20 : 10 : end]) = [];
    end
end

