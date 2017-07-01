function I_new = move_inertia_axis(I_old, m, pos_vec)
    I_new = I_old + m * (pos_vec.' * pos_vec * eye(3) - pos_vec * pos_vec.');
end