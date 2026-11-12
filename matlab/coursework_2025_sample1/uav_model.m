% Model of UAV
function [angles, angles_dot, angles_ddot] = uav_model(t,torque, angles_prev, angles_dot_prev)

    mu = 2.5e-3;
    dt = 0.01;

    M_bx = torque(1);
    M_by = torque(2);
    M_bz = torque(3);
    
    alpha_prev = angles_prev(1);
    betta_prev = angles_prev(2);
    gamma_prev = angles_prev(3);
    
    alpha_dot_prev = angles_dot_prev(1);
    betta_dot_prev = angles_dot_prev(2);
    gamma_dot_prev = angles_dot_prev(3);
    
    Sa = sin(alpha_prev);
    Ca = cos(alpha_prev);
    Sb = sin(betta_prev);
    Cb = cos(betta_prev);
    
    %acceleration angle of roll
    alpha_ddot = (M_bx + 0.3 * Sa * Cb - 2 * mu * alpha_dot_prev) / 0.02;
    
    denominator_betta = 0.344 + 0.028 * Ca^2 + 0.018 * Sa^2;
    
    %acceleration angle of pitch
    betta_ddot = (M_by * Ca - M_bz * Sa + 0.3 * Ca * Sb - 2 * mu * betta_dot_prev) / denominator_betta;
    
    denominator_gamma = 0.76 + 0.326 * Cb^2 + (0.026 * Sa^2 + 0.02 * Ca^2) * Cb^2;
    
    %acceleration angle of yaw
    gamma_ddot = (M_by * Sa * Cb + M_bz * Ca * Cb - 2 * mu * gamma_dot_prev) / denominator_gamma;
    
    %angle and velocity of roll
    alpha_dot = alpha_dot_prev + alpha_ddot * dt; 
    alpha = alpha_prev + alpha_dot_prev * dt;
    
    %angle and velocity of pitch
    betta_dot = betta_dot_prev + betta_ddot * dt; 
    betta = betta_prev + betta_dot_prev * dt;
    
    %angle and velocity of yaw
    gamma_dot = gamma_dot_prev + gamma_ddot * dt; 
    gamma = gamma_prev + gamma_dot_prev * dt;
    
    if any(isnan([alpha_ddot, betta_ddot, gamma_ddot]))
        fprintf('NaN detected at t=%.3f: alpha_ddot=%.6f, betta_ddot=%.6f, gamma_ddot=%.6f\n', ...
                t, alpha_ddot, betta_ddot, gamma_ddot);
        fprintf('M_bx=%.6f, M_by=%.6f, M_bz=%.6f\n', M_bx, M_by, M_bz);
        fprintf('Sa=%.6f, Ca=%.6f, Sb=%.6f, Cb=%.6f\n', Sa, Ca, Sb, Cb);
    end
    
    %returns
    angles = [alpha; betta; gamma];
    angles_dot = [alpha_dot; betta_dot; gamma_dot];
    angles_ddot = [alpha_ddot; betta_ddot; gamma_ddot];

end