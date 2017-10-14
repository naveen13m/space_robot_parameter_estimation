% This function computes the parameter coupling matrix which transforms the
% standard dynamic parameter vector to the minimal parameter vector
% Inputs: Minimal parameter vector (7n + 3 X 1)
%         Standard parameter vector (10n X 1)
%         Link mass vector (n X 1)
%         Link COM matrix (3 X n)
% Note: All inputs are required in symbolic form
% Output: Parameter coupling matrix
function pcm = compute_param_coupling_mat(minimal_param_vec, ...
                                             std_param_vec, mass, link_com)
    num_equs = length(minimal_param_vec);
    num_std_params = length(std_param_vec);
    pcm = sym(zeros(num_equs, num_std_params));
    for curr_equ_index = 1 : num_equs
        for curr_std_param_index = 1 : num_std_params
            curr_equ = minimal_param_vec(curr_equ_index, :);
            curr_std_param = std_param_vec(curr_std_param_index);
            is_mass_index = mod(curr_std_param_index, 10) == 7; 
            is_com_index = (mod(curr_std_param_index, 10) == 8) || ...
                           (mod(curr_std_param_index, 10) == 9) || ...
                           (mod(curr_std_param_index, 10) == 0);
            param_link_index = floor((curr_std_param_index - 1) / 10) + 1;
            if is_mass_index % Remove terms involving first moment
                all_coeffs = coeffs(curr_equ, curr_std_param);
                all_coeffs = simplify(subs(all_coeffs, ...
                                link_com(:, param_link_index), [0; 0; 0]));
            elseif is_com_index % Remove mass from first moment coeffs
                all_coeffs = coeffs(curr_equ, curr_std_param / mass(param_link_index));
                all_coeffs = all_coeffs / mass(param_link_index);
            else
                all_coeffs = coeffs(curr_equ, curr_std_param);
            end
            num_coeffs = length(all_coeffs);
            if num_coeffs == 1
                if all_coeffs ~= 1 % Current standard parameter (CSP) not present
                    dyn_param_coeff = sym(0);
                else % Equation carries only the CSP
                    dyn_param_coeff = all_coeffs;
                end
            else % Equation has CSP along with other terms
                dyn_param_coeff = all_coeffs(end);
            end
            pcm(curr_equ_index, curr_std_param_index) = ...
                                                           dyn_param_coeff;
        end
        fprintf('%0.2f%% ', curr_equ_index * 100 / num_equs)
    end
    fprintf('\n');
end