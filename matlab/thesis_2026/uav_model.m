function [alpha,alpha_dot,alpha_ddot] = uav_model(torque,alpha_prev,alpha_dot_prev)

mu = 2.5e-3;
global dt

% acceleration
alpha_ddot = (torque + 0.3*sin(alpha_prev) - 2*mu*alpha_dot_prev)/0.02;

% integration
alpha_dot = alpha_dot_prev + alpha_ddot*dt;
alpha = alpha_prev + alpha_dot_prev*dt + alpha_ddot*dt*dt/2;

end