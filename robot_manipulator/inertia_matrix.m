function M = inertia_matrix(theta, p)
    % calculating the matrix M(theta)
    theta2 = theta(2);
    theta3 = theta(3);
    
    M = zeros(3,3);
    
    % M11
    M(1,1) = p.I1 + p.m2 * p.lg2^2 * sin(theta2)^2 + p.m3 * (p.l2 * sin(theta2) + p.lg3 * sin(theta2 + theta3))^2;
    
    % M22 
    M(2,2) = p.I2 + p.m2 * p.lg2^2 + p.m3 * p.l2^2 + p.I3 + p.m3 * p.lg3^2 + 2 * p.m3 * p.l2 * p.lg3 * cos(theta3);
    
    % M33
    M(3,3) = p.I3 + p.m3 * p.lg3^2;
    
    % M23 = M32
    M(2,3) = p.I3 + p.m3 * p.lg3^2 + p.m3 * p.l2 * p.lg3 * cos(theta3);
    M(3,2) = M(2,3);
end