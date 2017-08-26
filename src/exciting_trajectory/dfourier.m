function [dfourier_val] = dfourier(tr_par, curr_t)
    num_harmonics = (length(tr_par) - 2) / 2;
    dfourier_val = 0;
    om = tr_par(2);
    cos_par = tr_par(3 : 1 : num_harmonics + 2);
    sin_par = tr_par(num_harmonics + 3 : 1 : end);
    for har_index = 1 : num_harmonics
        dfourier_val = dfourier_val ...
          + har_index * om * cos_par(har_index) * cos(har_index * om * curr_t) ...
          - har_index * om * sin_par(har_index) * sin(har_index * om * curr_t);
    end
end

