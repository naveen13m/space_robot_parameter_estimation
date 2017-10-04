% Converts GKM to RM and Output vector
function [reg_mat, output_vec] = reduce_gkm(gkm, is_planar, rw_params)
    if isempty(rw_params)
        if is_planar
            reg_mat = eliminate_dependency(gkm);
            output_vec = zeros(size(reg_mat, 1), 1);
        else
            reg_mat = gkm;
            reg_mat(:, [12 : 10 : end, 17 : 10 : end, 20 : 10 : end]) = [];
            output_vec = zeros(size(reg_mat, 1), 1);
        end
    else
        Ixx = rw_params.Ixx; Iyy = rw_params.Iyy; Izz = rw_params.Izz;
        Ixy = rw_params.Ixy; Iyz = rw_params.Iyz; Izx = rw_params.Izx;
        m = rw_params.m;
        x_com = rw_params.x_com; y_com = rw_params.y_com; z_com = rw_params.z_com;
        rw_param = [Ixx; Iyy; Izz; Ixy; Iyz; Izx; m; m * x_com; m * y_com; m * z_com];
        if is_planar
            reg_mat = gkm(:, 1 : end - 10);
            reg_mat = eliminate_dependency(reg_mat);
            all_rw_param = rw_param;
            output_vec = -gkm(:, end - 9 : end) * all_rw_param;
            output_vec([3 : 6 : end, 4 : 6 : end, 5 : 6 : end], :) = [];
        else
            all_rw_param = [rw_param; rw_param; zeros(10, 1); rw_param];
            output_vec = -gkm(:, end - 39 : end) * all_rw_param;
            reg_mat = gkm(:, 1 : end - 40);
            reg_mat(:, [12 : 10 : end, 17 : 10 : end, 20 : 10 : end]) = [];
        end
    end
end

