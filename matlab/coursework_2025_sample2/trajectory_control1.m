% Trajectory control of uav
function u = trajectory_control1(t, alpha)
    
    [alpha_desired, alpha_dot_desired, ~] = desired_trajectory1(t);
    
    a = -0.3;    
    phi = alpha - alpha_desired;
    
    u = 0.005 * (alpha_dot_desired + a * phi);
     
end