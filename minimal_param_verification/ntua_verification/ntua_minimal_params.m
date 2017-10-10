function error = ntua_minimal_params(x)
    % 3-LINK planar system
    M = sum(x(1 : 3));
    m0 = x(1); m1 = x(2); m2 = x(3);
    r0x = x(4); r0y = x(5); r1 = x(6); r2 = x(7);
    l1 = 1 - r1; l2 = 1 - r2;
    I0 = x(8); I1 = x(9); I2 = x(10);
        
    pi(1) = (m0 / M) * r0x * (l1 * (m1 + m2) + r1 * m2);
    pi(2) = (l2 * m0 * m2 / M) * r0x;
    pi(3) = (m0 / M) * r0y * (l1 * (m1 + m2) + r1 * m2);
    pi(4) = (l2 * m0 * m2 / M) * r0y;
    pi(5) = I0 + (m0 * (m1 + m2) / M) * (r0x^2 + r0y^2);
    pi(6) = (l2 * m2 / M) * (l1 * m0 + r1 * (m0 + m1));
    pi(7) = I1 + (l1^2 * m0 * (m1 + m2) + 2 * l1 * m0 * m2 * r1 + (m0 + m1) * m2 * r1^2) / M;
    pi(8) = I2 + l2^2 * (m0 + m1) * m2 / M;
    
    actual_minimal_param = [62.5,20.83,0,0,1241.67,43.75,156.25,72.9167];
    error = norm(actual_minimal_param - pi);
end