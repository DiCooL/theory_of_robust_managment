% Desired trajectory
function [angles_desired, angles_dot_desired, angles_ddot_desired] = desired_trajectory(t)

    A = pi/6;           
    omega = pi/10;

    % Desired angles
    alpha_desired = A * sin(omega * t);% * (t < 5) + A * sin(omega * 5) * (t >= 5); 
    betta_desired = 0;
    gamma_desired = 0;
        
    % Desired velocity
    alpha_dot_desired = A * omega * cos(omega * t);% * (t < 5);
    betta_dot_desired = 0;
    gamma_dot_desired = 0;
    
    % Desired accelerations
    alpha_ddot_desired = - A * omega^2 *sin(omega * t);%* (t < 5);
    betta_ddot_desired = 0;
    gamma_ddot_desired = 0;

    angles_desired = [alpha_desired; betta_desired; gamma_desired];
    angles_dot_desired = [alpha_dot_desired; betta_dot_desired; gamma_dot_desired];
    angles_ddot_desired = [alpha_ddot_desired; betta_ddot_desired; gamma_ddot_desired];
end