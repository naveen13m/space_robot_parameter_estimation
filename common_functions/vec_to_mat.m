% Converts a vector to a skew symmetric matrix
% Inputs: Vector (3 X 1)
% Outputs: Skew symmetric matrix (3 X 3)
function skew_symm_matrix = vec_to_mat(vector)
    skew_symm_matrix = [0          -vector(3)  vector(2); ...
                        vector(3)   0         -vector(1); ...
                       -vector(2)  vector(1)  0];
end