% Given the parameters of individual links, it computes the value of coupled
% parameters for a 3-link planar system.
function pi = compute_coupled_params(x)
    % 3-Link planar system
    Izz0c = x(1); m0 = x(2); a0x = x(3); a0y = x(4);
    Izz1c = x(5); m1 = x(6); a1x = x(7); a1y = x(8);
    Izz2c = x(9); m2 = x(10); a2x = x(11); a2y = x(12);
    Izz0 = Izz0c + m0 * norm([a0x; a0y])^2;
    Izz1 = Izz1c + m1 * norm([a1x; a1y])^2;
    Izz2 = Izz2c + m2 * norm([a2x; a2y])^2;
    
    pi(1) = Izz0 + 0.25 * (m1 + m2);
    pi(2) = m0 + m1 + m2;
    pi(3) = m0 * a0x + 0.5 * (m1 + m2);
    pi(4) = m0 * a0y;
    pi(5) = Izz1 + m2;
    pi(6) = m1 * a1x + m2;
    pi(7) = m1 * a1y;
    pi(8) = Izz2;
    pi(9) = m2 * a2x;
    pi(10) = m2 * a2y;
end

