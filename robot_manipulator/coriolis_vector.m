function h_vec = coriolis_vector(theta, theta_dot, p)
    %h(theta, theta_dot, p)
    
    theta1 = theta(1);
    theta2 = theta(2);
    theta3 = theta(3);
    
    theta1_dot = theta_dot(1);
    theta2_dot = theta_dot(2);
    theta3_dot = theta_dot(3);
    
    h_vec = zeros(3,1);
    
    % h1 
    h_vec(1) = 2 * p.m2 * p.lg2^2 * sin(theta2) * cos(theta2) + ...
           2 * p.m3 * (p.l3 * sin(theta2) + p.lg3 * sin(theta2 + theta3)) * p.l2 * cos(theta2) + ...
           2 * p.m3 * (p.l2 * sin(theta2) + p.lg3 * sin(theta2 + theta3)) * p.lg3 * cos(theta2 + theta3) * theta1_dot * theta2_dot + ...
           2 * p.m3 * (p.l2 * sin(theta2) + p.lg3 * sin(theta2 + theta3)) * p.lg3 * cos(theta2 + theta3) * theta1_dot * theta3_dot;

    % h2 
    h_vec(2) = -2 * p.m3 * p.l2 * p.lg3^2 * sin(theta3) * theta2_dot * theta3_dot - ...
           p.m3 * p.l2 * p.lg3 * sin(theta3) * theta3_dot^2 - ...
           p.m2 * p.lg2^2 * sin(theta2) + p.m3 * (p.l2 * sin(theta2) + p.lg3 * sin(theta2 + theta3)) * (p.l2 * cos(theta2) + p.lg3 * cos(theta2 + theta3)) * theta1_dot^2;

    % h3 
    h_vec(3) = -p.m3 * (p.l2 * sin(theta2) + p.lg3 * sin(theta2 + theta3)) * p.lg3 * cos(theta2 + theta3) * theta1_dot^2 + ...
           p.m3 * p.l2 * p.lg3 * sin(theta3) * theta2_dot^2;
end