% Launching the uav
clear all
clc

% Initial states
t = 0; 
angles = [0; 0; 0]; 
angles_dot = [pi^2/60; 0; 0];
angles_ddot = [0; 0; 0];

dt = 0.01; % it is also located in uav_model
t_final = 10;
n_steps = round(t_final / dt);

% Arrays for storing results
angles_history = zeros(3, n_steps);
angles_dot_history = zeros(3, n_steps);
angles_ddot_history = zeros(3, n_steps);
angles_desired_history = zeros(3, n_steps);
angles_dot_desired_history = zeros(3, n_steps);
time_history = zeros(1, n_steps);

% Arrays for control signals
torque_history = zeros(3, n_steps);
u_history = zeros(3, n_steps);
u_wave_history = zeros(3, n_steps);

f = waitbar(0, 'Simulation running...');

k = 1;
torque = [pi/6 * sin(pi/10 * dt); 0; 0];
while t < t_final
    
    % Control signals
    u_fil = 1 * torque; % filter corrector
    u_wave = linear_model(angles, angles_dot, angles_ddot);
    u_del = u_wave - u_fil;
    
    [u, angles_desired, angles_dot_desired] = trajectory_control_prev(t, angles, angles_dot);
    torque = u - u_del;
    
    % Save control signals
    torque_history(:, k) = torque;
    u_history(:, k) = u;
    u_wave_history(:, k) = u_wave;
    
    % Dynamics
    [angles_new, angles_dot_new, angles_ddot_new] = uav_model(t,torque, angles, angles_dot);
    
    [angles_desired, angles_dot_desired, angles_ddot_desired] = desired_trajectory(t);

    % Save results
    angles_history(:, k) = angles_new;
    angles_dot_history(:, k) = angles_dot_new;
    angles_ddot_history(:, k) = angles_ddot_new;
    angles_desired_history(:, k) = angles_desired;
    angles_dot_desired_history(:, k) = angles_dot_desired;
    time_history(k) = t;
    
    % Update state
    angles = angles_new; 
    angles_dot = angles_dot_new;
    angles_ddot = angles_ddot_new;
    t = t + dt;
    k = k + 1;
    
    % Progress
    waitbar(t/t_final, f, sprintf('Progress: %.1f%%', t/t_final*100));
    
end

close(f);

% Trim arrays to actual number of steps
angles_history = angles_history(:, 1:k-1);
angles_dot_history = angles_dot_history(:, 1:k-1);
angles_ddot_history = angles_ddot_history(:, 1:k-1);
angles_desired_history = angles_desired_history(:, 1:k-1);
angles_dot_desired_history = angles_dot_desired_history(:, 1:k-1);
torque_history = torque_history(:, 1:k-1);
u_history = u_history(:, 1:k-1);
u_wave_history = u_wave_history(:, 1:k-1);
time_history = time_history(1:k-1);

% PLOTTING
figure('WindowState', 'maximized')

% Angles
%subplot(3,3,1)
subplot(1,2,1)
hold all
plot(time_history, angles_history(1,:)*180/pi, 'r', 'linewidth', 1)
%plot(time_history, angles_history(2,:)*180/pi, 'g', 'linewidth', 1)
%plot(time_history, angles_history(3,:)*180/pi, 'b', 'linewidth', 1)
plot(time_history, angles_desired_history(1,:)*180/pi, 'r--', 'linewidth', 1)
%plot(time_history, angles_desired_history(2,:)*180/pi, 'g--', 'linewidth', 1)
%plot(time_history, angles_desired_history(3,:)*180/pi, 'b--', 'linewidth', 1)
title('Angles')
xlabel('Time, s')
ylabel('Angle, deg')
%legend('Alpha', 'Beta', 'Gamma', 'Alpha desired', 'Beta desired', 'Gamma desired')
grid on

% Angular velocities
%subplot(3,3,2)
subplot(1,2,2)
hold all
plot(time_history, angles_dot_history(1,:)*180/pi, 'r', 'linewidth', 1)
%plot(time_history, angles_dot_history(2,:)*180/pi, 'g', 'linewidth', 1)
%plot(time_history, angles_dot_history(3,:)*180/pi, 'b', 'linewidth', 1)
plot(time_history, angles_dot_desired_history(1,:)*180/pi, 'r--', 'linewidth', 1)
%plot(time_history, angles_dot_desired_history(2,:)*180/pi, 'g--', 'linewidth', 1)
%plot(time_history, angles_dot_desired_history(3,:)*180/pi, 'b--', 'linewidth', 1)
title('Angular Velocities')
xlabel('Time, s')
ylabel('Angular Velocity, deg/s')
%legend('Alpha dot', 'Beta dot', 'Gamma dot', 'Alpha dot desired', 'Beta dot desired', 'Gamma dot desired')
grid on
box on
%{
% Angular accelerations
subplot(3,3,3)
hold all
plot(time_history, angles_ddot_history(1,:)*180/pi, 'r', 'linewidth', 1)
plot(time_history, angles_ddot_history(2,:)*180/pi, 'g', 'linewidth', 1)
plot(time_history, angles_ddot_history(3,:)*180/pi, 'b', 'linewidth', 1)
title('Angular Accelerations')
xlabel('Time, s')
ylabel('Angular Acceleration, deg/s?')
legend('Alpha ddot', 'Beta ddot', 'Gamma ddot')
grid on
box on

% Control moments
subplot(3,3,4)
hold all
plot(time_history, torque_history(1,:), 'r', 'linewidth', 1)
plot(time_history, torque_history(2,:), 'g', 'linewidth', 1)
plot(time_history, torque_history(3,:), 'b', 'linewidth', 1)
title('Control Moments')
xlabel('Time, s')
ylabel('Torque, N?m')
legend('M_{bx}', 'M_{by}', 'M_{bz}')
grid on
box on

% Control signals from trajectory controller
subplot(3,3,5)
hold all
plot(time_history, u_history(1,:), 'r', 'linewidth', 1)
plot(time_history, u_history(2,:), 'g', 'linewidth', 1)
plot(time_history, u_history(3,:), 'b', 'linewidth', 1)
title('Trajectory Control Signals')
xlabel('Time, s')
ylabel('Control Signal')
legend('u_{alpha}', 'u_{beta}', 'u_{gamma}')
grid on
box on

% Linear model outputs
subplot(3,3,6)
hold all
plot(time_history, u_wave_history(1,:), 'r', 'linewidth', 1)
plot(time_history, u_wave_history(2,:), 'g', 'linewidth', 1)
plot(time_history, u_wave_history(3,:), 'b', 'linewidth', 1)
title('Linear Model Outputs')
xlabel('Time, s')
ylabel('u_{wave}')
legend('u_{wave,x}', 'u_{wave,y}', 'u_{wave,z}')
grid on
box on

% Angle errors
subplot(3,3,7)
hold all
angle_errors = angles_history - angles_desired;
plot(time_history, angle_errors(1,:)*180/pi, 'r', 'linewidth', 1)
plot(time_history, angle_errors(2,:)*180/pi, 'g', 'linewidth', 1)
plot(time_history, angle_errors(3,:)*180/pi, 'b', 'linewidth', 1)
title('Angle Errors (Absolute)')
xlabel('Time, s')
ylabel('Angle Error, deg')
legend('Alpha error', 'Beta error', 'Gamma error')
grid on
box on

% 3D trajectory of angles (phase portrait)
subplot(3,3,8)
plot3(angles_history(1,:)*180/pi, angles_history(2,:)*180/pi, angles_history(3,:)*180/pi, 'b', 'linewidth', 1)
hold on
plot3(0, 0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r')
title('3D Angle Trajectory')
xlabel('Alpha, deg')
ylabel('Beta, deg')
zlabel('Gamma, deg')
grid on
box on


% Total error norm
subplot(3,3,9)
hold all
error_norm = sqrt(sum(angle_errors.^2, 1))*180/pi;
semilogy(time_history, error_norm, 'k', 'linewidth', 1)
title('Total Error Norm')
xlabel('Time, s')
ylabel('||error||, deg')
grid on

fprintf('Final angles: alpha = %.4f, beta = %.4f, gamma = %.4f\n', ...
        angles_history(1,end)*180/pi, angles_history(2,end)*180/pi, angles_history(3,end)*180/pi);
fprintf('Final angular velocities: alpha_dot = %.4f, beta_dot = %.4f, gamma_dot = %.4f\n', ...
        angles_dot_history(1,end)*180/pi, angles_dot_history(2,end)*180/pi, angles_dot_history(3,end)*180/pi);
%}