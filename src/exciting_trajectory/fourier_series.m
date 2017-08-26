function [fourier_val] = fourier_series(tr_par, curr_t)
    num_harmonics = (length(tr_par) - 2) / 2;
    fourier_val = tr_par(1);
    om = tr_par(2);
    sin_par = tr_par(3 : 1 : num_harmonics + 2);
    cos_par = tr_par(num_harmonics + 3 : 1 : end);
    for har_index = 1 : num_harmonics
        fourier_val = fourier_val ...
          + sin_par(har_index) * sin(har_index * om * curr_t) ...
          + cos_par(har_index) * cos(har_index * om * curr_t);
    end
end