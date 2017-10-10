% Computes the error norm of the coupled parameters of the actual
% parameters and the guessed parameters.
function [error_norm] = error_coupled_params(x)
    global actual_params;
    actual_coupled_params = compute_coupled_params(actual_params);
%     actual_coupled_params = [529.9874  1080  40.0053    0.0001   59.9984   55.0002    0.0000   10.0000   14.9989      0];
    coupled_param = compute_coupled_params(x);
    param_error = actual_coupled_params - coupled_param;
    error_norm = norm(param_error)^2;
end

