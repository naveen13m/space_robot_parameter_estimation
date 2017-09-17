function [reg_mat, output_vec] = reduce_gkm(gkm, is_planar, rw_params)
    Ixx = rw_params.Ixx; Iyy = rw_params.Iyy; Izz = rw_params.Izz;
    Ixy = rw_params.Ixx; Iyz = rw_params.Iyz; Izx = rw_params.Izx;
    m = rw_params.m;
    x_com = rw_params.x_com; y_com = rw_params.y_com; z_com = rw_params.z_com;
    if is_planar
        output_vec = -gkm(:, end - 9 : end) * ...
            [Ixx; Iyy; Izz; Ixy; Iyz; Izx; m; m * x_com; m * y_com; m * z_com];
        reg_mat = gkm(:, 1 : end - 10);
        reg_mat = eliminate_dependency(reg_mat);
    else
        output_vec = -gkm(:, end - 9 : end) * ...
            [Ixx; Iyy; Izz; Ixy; Iyz; Izx; m; m * x_com; m * y_com; m * z_com];
        reg_mat = gkm(:, 1 : end - 10);
        reg_mat(:, [12 : 10 : end, 17 : 10 : end, 20 : 10 : end]) = [];
    end
end

