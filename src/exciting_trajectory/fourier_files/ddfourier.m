function [ddfourier_val] = ddfourier(tr_par, curr_t)
    num_harmonics = (length(tr_par) - 2) / 2;
    ddfourier_val = 0;
    om = tr_par(2);
    sin_par = tr_par(3 : 1 : num_harmonics + 2);
    cos_par = tr_par(num_harmonics + 3 : 1 : end);
    for har_index = 1 : num_harmonics
        ddfourier_val = ddfourier_val ...
            - (har_index * om)^2 * sin_par(har_index) * sin(har_index * om * curr_t) ...
            - (har_index * om)^2 * cos_par(har_index) * cos(har_index * om * curr_t);
    end
    nullifier_index = num_harmonics + 1;
    vel_nullifier = -(nullifier_index * ([1 : 1 : num_harmonics] * sin_par.') * om^2) * ...
                            sin(nullifier_index * om * curr_t);
    ddfourier_val = ddfourier_val - vel_nullifier;
end