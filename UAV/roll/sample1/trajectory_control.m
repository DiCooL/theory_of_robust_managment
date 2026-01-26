% Trajectory control of uav
function u = trajectory_control(t, angles, angles_dot, angles_ddot)
    
    [angles_desired, angles_dot_desired, angles_ddot_desired] = desired_trajectory(t);
    
    A = pi/6;           
    omega = pi/10;
    
    alpha = A * sin(omega * t);
    betta = 0;
    gamma = 0;
    
    alpha_dot = A * omega * cos(omega * t);
    betta_dot = 0;
    gamma_dot = 0;
    
    alpha_ddot = - A * omega^2 *sin(omega * t);
    betta_ddot = 0;
    gamma_ddot = 0;
    
    mu = 2.5e-3;
    
    %triga
    s_a = sin(alpha);
    c_a = cos(alpha);
    s_b = sin(betta);
    c_b = cos(betta);
    
    %denominators of uav model
    K1 = 0.344 + 0.028 * c_a^2 + 0.018 * s_a^2;
    K2 = 0.76 + 0.326 * c_b^2 + (0.026 * s_a^2 + 0.02 * c_a^2) * c_b^2;
    
    %without torque x y z
    D1 = K1 * betta_ddot + 2 * mu * betta_dot - 0.3 * c_a * s_b;
    D2 = K2 * gamma_ddot + 2 * mu * gamma_dot;
        
    %M_bx
    M_bx = 0.02 * alpha_ddot + 2 * mu * alpha_dot - 0.3 * s_a * c_b;
    
    % M_by & M_bz
    %{
    if abs(c_b) < 1e-12
        M_by = 0;
        M_bz = 0;
    else
        M_by = (D1 * c_a * c_b + D2 * s_a) / c_b;
        M_bz = (c_a * D2 - s_a * c_b * D1) / c_b;
    end
    %}
    M_by = (D1 * c_a * c_b + D2 * s_a) / c_b;
    M_bz = (c_a * D2 - s_a * c_b * D1) / c_b;
    
    % add feedback establish on errors
    phi = angles - angles_desired;
    phi_dot = angles_dot - angles_dot_desired;
    
    % PD-regulation
    kp_alpha = 10;  
    kd_alpha = 5;
    kp_betta = 0;
    kd_betta = 0;
    kp_gamma = 0;
    kd_gamma = 0;
    
    % 
    u_feedback = -[kp_alpha * phi(1) + kd_alpha * phi_dot(1);
                   kp_betta * phi(2) + kd_betta * phi_dot(2);
                   kp_gamma * phi(3) + kd_gamma * phi_dot(3)];
         
    %returns
    u = [M_bx; M_by; M_bz];%+ u_feedback;
     
end