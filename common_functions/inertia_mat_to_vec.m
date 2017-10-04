% Converts a symmetric matrix to a vector
% Inputs: Inertia matrix (3 X 3) *
% Ouputs: Inertia vector (6 X 1) *
% *can be both symbolic and numeric
function I_vec = inertia_mat_to_vec(I)
    I_vec = [I(1, 1) I(2, 2) I(3, 3) I(1, 2) I(2, 3) I(3, 1)].'; 
end