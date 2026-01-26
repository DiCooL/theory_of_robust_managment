% Linear model of UAV
function u_wave = linear_model(angles, angles_dot, angles_ddot)
    
    alpha = angles(1);
    betta = angles(2);
    gamma = angles(3);
    
    alpha_dot = angles_dot(1);
    betta_dot = angles_dot(2);
    gamma_dot = angles_dot(3);
    
    alpha_ddot = angles_ddot(1);
    betta_ddot = angles_ddot(2);
    gamma_ddot = angles_ddot(3);
    
    Sa = sin(alpha);
    Ca = cos(alpha);
    Sb = sin(betta);
    Cb = cos(betta);
    
    mu = 2.5e-3;
    
    M_bx = 0.02 * alpha_ddot - 0.3 * alpha * 1 + 2 * mu * alpha_dot;
    
    denominator_gamma = 0.76 + 0.33 * Cb^2;
    rhs2 = 0.37 * betta_ddot - 0.3 * Ca * Sb + 2 * mu * betta_dot;    
    rhs3 = denominator_gamma * gamma_ddot + 2 * mu * gamma_dot;
    
    epsilon = 1e-10;  
    
    if abs(Sa) < epsilon
        M_by = rhs2 / Ca;
        M_bz = rhs3 / (Ca * Cb);
    else
        M_by = rhs3 * Sa / Cb + rhs2 * Ca;
        M_bz = (M_by * Ca - rhs2) / Sa;
    end
    
    %M_by = rhs3 * Sa / Cb + rhs2 * Ca;
    %M_bz = (M_by * Ca - rhs2) / Sa;
    
    u_wave = [M_bx; M_by; M_bz];

end