% Trajectory control of uav
function [u, angles_desired, angles_dot_desired] = trajectory_control_prev(t, angles, angles_dot)
    
    alpha = angles(1);
    betta = angles(2);
    gamma = angles(3);
    
    alpha_dot = angles_dot(1);
    betta_dot = angles_dot(2);
    gamma_dot = angles_dot(3);
    
    % Desired angles
    A = pi/6;           
    omega = pi/10;
    alpha_desired = A * sin(omega * t);
    
    betta_desired = 0;
    gamma_desired = 0;
        
    % Desired velocity
    alpha_dot_desired = A * omega * cos(omega * t);
    betta_dot_desired = 0;
    gamma_dot_desired = 0;
        
    % Coefficient of PD-relulation (experimental)
    Kp_alpha = 2.0;   % torue/rad
    Kd_alpha = 0.5;   % torque/(rad/s)
    
    Kp_betta = 0.4;
    Kd_betta = 0.08;
    
    Kp_gamma = 0.3;
    Kd_gamma = 0.06;
    
    % PD-relulation: calculate the desired control moments
    M_bx_desired = Kp_alpha * (alpha_desired - alpha) + Kd_alpha * (alpha_dot_desired - alpha_dot);
    M_by_desired = Kp_betta * (betta_desired - betta) + Kd_betta * (betta_dot_desired - betta_dot);
    M_bz_desired = Kp_gamma * (gamma_desired - gamma) + Kd_gamma * (gamma_dot_desired - gamma_dot);
    
    %returns
    u = [M_bx_desired; M_by_desired; M_bz_desired];
    angles_desired = [alpha_desired; betta_desired; gamma_desired];
    angles_dot_desired = [alpha_dot_desired; betta_dot_desired; gamma_dot_desired];

        
end