function g_vec = gravity_vector(theta, p)
    
    theta2 = theta(2);
    theta3 = theta(3);
    
    g_vec = zeros(3,1);
    
    g_vec(1) = 0;
    
    % g2
    g_vec(2) = -(p.m2 * p.lg2 + p.m3 * p.l2) * p.g * sin(theta2) - ...
                p.m3 * p.lg3 * p.g * sin(theta2 + theta3);
    
    % g3  
    g_vec(3) = -p.m3 * p.lg3 * p.g * sin(theta2 + theta3);
end