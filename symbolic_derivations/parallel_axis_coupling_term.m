% Parallel axis theorem moves the inertia about the COM to the desired
% location. Howeever, to move the frame from a generic location, extra
% terms come into play. This is one of the coupling terms.
clear all; close all; clc;

syms lx ly lz
l = [lx ly lz].';
syms ax ay az
a = [ax ay az].';
syms r1 r2 r3 r4 r5 r6 r7 r8 r9
R = [r1 r2 r3; r4 r5 r6; r7 r8 r9];

term = simplify(2 * l.' * R * a * eye(3) - (l * (R * a).' + R * a * l.'));
term_vec = inertia_mat_to_vec(term);

mat = sym(zeros(6, 3));

for com_term_index = 1 : 3
    for coup_term_index = 1 : 6
        all_coeffs = coeffs(term_vec(coup_term_index), a(com_term_index));
        if length(all_coeffs) > 1
            coeff = all_coeffs(2);
        end
        mat(coup_term_index, com_term_index) = coeff;
    end
end

mat
simple_mat = subs(mat, [r1 r2 r3, r4 r5 r6, r7 r8 r9], [1 0 0, 0 1 0, 0 0 1])