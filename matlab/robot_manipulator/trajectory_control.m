function u = trajectory_control(t, theta)
    
    p = robot_parameters;
    JJ = jacobian_matrix(theta);
    [phi, Q] = phi_Q(t, theta);
    
    %regularization
    regular = 1e-6 * eye(3); 
    if any(isnan(JJ(:))) || any(isinf(JJ(:)))
        JJ = regular;
    end
    if any(isnan(phi(:))) || any(isinf(phi(:)))
        phi = [1e-6; 1e-6; 1e-6];
    end
        
    %u = [JJ*K^-1]^-1 * [alpha*phi - Q]
    u = (1e-6*eye(3,3) + JJ / p.K) \ (p.alpha * phi - Q); %[3 x 3] \ [3 x 1]
    
end