% Model of UAV
function [alpha, alpha_dot, alpha_ddot] = uav_model1(torque, alpha_prev, alpha_dot_prev)

    mu = 2.5e-3;
    dt = 0.001;

    M_bx = torque;   
    Sa = sin(alpha_prev);
    
    %acceleration 
    alpha_ddot = (M_bx + 0.3 * Sa - 2 * mu * alpha_dot_prev) / 0.02;
    
    %angle and velocity of roll
    alpha_dot = alpha_dot_prev + alpha_ddot * dt; 
    alpha = alpha_prev + alpha_dot_prev * dt;
       
end