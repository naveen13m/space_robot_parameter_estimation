clear all; close all; clc;

syms I11 I22 I33 I12 I23 I31
I = [I11 I12 I31;
I12 I22 I23;
I31 I23 I33];
syms al thi
iRip1 = DH_to_rot_mat(al, thi);
I_mod = simplify(iRip1 * I * iRip1.');
I_vec = inertia_mat_to_vec(I);
I_mod_vec = inertia_mat_to_vec(I_mod);

coeffs_mat = sym(zeros(6, 6));

for curr_row = 1 : 6
    for curr_element = 1 : 6
        equation = I_mod_vec(curr_row);
        variable = I_vec(curr_element);
        all_coeffs = coeffs(equation, variable);
        if length(all_coeffs) > 1
            coeffs_mat(curr_row, curr_element) = all_coeffs(2);
        end
    end
end
coeffs_mat = simplify(coeffs_mat)

% verification
% is_zero = simplify(I_mod_vec - coeffs_mat * I_vec)