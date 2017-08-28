function [th_d, dth_d, ddth_d]=trajectory(t, n, tf, tr_par)
    num_params = length(tr_par) / (n - 1);
    tr_par = reshape(tr_par, num_params, n - 1).';
    th_d = zeros(n-1,1); dth_d = zeros(n-1,1); ddth_d = zeros(n-1,1); 
    for i=1:n-1
        th_d(i, 1) = fourier_series(tr_par(i,:), t);
        dth_d(i, 1)= dfourier(tr_par(i,:), t);
        ddth_d(i, 1)= ddfourier(tr_par(i,:), t);
    end
end