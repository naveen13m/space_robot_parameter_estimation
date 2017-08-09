function I_vec = inertia_mat_to_vec(I)
    I_vec = [I(1, 1) I(2, 2) I(3, 3) I(1, 2) I(2, 3) I(3, 1)].'; 
end