% Desired trajectory
function [alpha_desired, alpha_dot_desired, alpha_ddot_desired] = desired_trajectory1(t)

    A = pi/6;           
    omega = pi/10;

    % Desired angles
    alpha_desired = A * sin(omega * t);%*(t<=10) + A * sin(omega * 10)*(t>10);
            
    % Desired velocity
    alpha_dot_desired = A * omega * cos(omega * t);
    
    % Desired accelerations
    alpha_ddot_desired = - A * omega^2 *sin(omega * t);
end