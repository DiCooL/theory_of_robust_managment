function [phi, Q] = phi_Q(t, theta)   

    p = robot_parameters;
    
    % current coordinate end manipulator
    [X_real, Y_real, Z_real] = forward_kinematics(theta);
    [X, Y, Z] = desired_kinematics(t);
    
    %{
    phi = [X - p.a - p.r * sin(p.omega * t);
           Y - p.b;
           Z - p.c - p.r * cos(p.omega * t)];
    
    % desired velocity
    Q = [-p.r * p.omega * cos(p.omega * t);
         0;
         p.r * p.omega * sin(p.omega * t)];
    %}
    
    phi = [X_real - X;
           Y_real - Y;
           Z_real - Z];
    
    Q = [p.r * p.omega * sin(p.omega * t);
         0;
         p.r * p.omega * cos(p.omega * t)];
    
end
