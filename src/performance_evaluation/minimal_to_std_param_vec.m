% This function computes one of the multiple standard parameter vector (SPV)
% of a given minimal parameter vector. Given the solutions of the redundant
% elements, this calculates the remaining elements in SPV
% Inputs: Parameter coupling matrix (num_minimal_params X num_std_params)
%         Minimal parameter vector (num_minimal_params X 1)    
%         Redundant param value (redundancy X 1)
%         Redundant param index (1 X redundancy)
% Outputs: Standard parameter vector (num_std_params X 1)
function std_param_vec = minimal_to_std_param_vec(param_coup_mat, minimal_param_vec, ...
                              redundant_param_value, redundant_param_index)
    num_cols_coup_mat = size(param_coup_mat, 2);
    std_param_vec = zeros(num_cols_coup_mat, 1);
    std_param_vec(redundant_param_index) = redundant_param_value;
    comp_param_index = setdiff(1 : 1 : num_cols_coup_mat, redundant_param_index);
    out_vec = minimal_param_vec - param_coup_mat(:, redundant_param_index) * redundant_param_value;
%     rank(param_coup_mat(:, comp_param_index))
%     std_param_vec = rref(param_coup_mat(:, comp_param_index));
    std_param_vec(comp_param_index) = pinv(param_coup_mat(:, comp_param_index)) * out_vec;
end

