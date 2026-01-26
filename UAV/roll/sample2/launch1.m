% Launching the uav
clear all
clc

% Initial states
t = 0; 
alpha = 0; 
alpha_dot = pi^2/60;
alpha_ddot = 0;

dt = 0.001; % it is also located in uav_model
t_final = 2000;
n_steps = round(t_final / dt);

% Arrays for storing results
alpha_history = zeros(1, n_steps);
alpha_dot_history = zeros(1, n_steps);
alpha_ddot_history = zeros(1, n_steps);
alpha_desired_history = zeros(1, n_steps);
alpha_dot_desired_history = zeros(1, n_steps);
time_history = zeros(1, n_steps);

% Arrays for control signals
torque_history = zeros(1, n_steps);
u_history = zeros(1, n_steps);
u_wave_history = zeros(1, n_steps);

f = waitbar(0, 'Simulation running...');

k = 1;
u_fil = 0;
alpha_f = 0.95;
torque = 0;
while t < t_final 
    
    % Control signals
    u_fil = 1 * torque; % filter corrector
    u_wave = linear_model1(alpha_dot);
    u_del = u_wave - u_fil;
    
    u = trajectory_control1(t, alpha);
    torque = u - u_del;
    
    % Save control signals
    torque_history(k) = torque;
    u_history(k) = u;
    u_wave_history(k) = u_wave;
    
    % Dynamics
    [alpha_new, alpha_dot_new, alpha_ddot_new] = uav_model1(torque, alpha, alpha_dot);   

    % Save results
    alpha_history(k) = alpha_new;
    alpha_dot_history(k) = alpha_dot_new;
    alpha_ddot_history(k) = alpha_ddot_new;
    
    [alpha_desired, alpha_dot_desired, alpha_ddot_desired] = desired_trajectory1(t);
    alpha_desired_history(:, k) = alpha_desired;
    alpha_dot_desired_history(:, k) = alpha_dot_desired;
    
    time_history(k) = t;
    
    % Update state
    alpha = alpha_new; 
    alpha_dot = alpha_dot_new;
    alpha_ddot = alpha_ddot_new;
    t = t + dt;
    k = k + 1;
    
    % Progress
    waitbar(t/t_final, f, sprintf('Progress: %.1f%%', t/t_final*100));
    
end

close(f);

% Trim arrays to actual number of steps
alpha_history = alpha_history(:, 1:k-1);
alpha_dot_history = alpha_dot_history(:, 1:k-1);
alpha_ddot_history = alpha_ddot_history(:, 1:k-1);
alpha_desired_history = alpha_desired_history(:, 1:k-1);
alpha_dot_desired_history = alpha_dot_desired_history(:, 1:k-1);
torque_history = torque_history(:, 1:k-1);
u_history = u_history(:, 1:k-1);
u_wave_history = u_wave_history(:, 1:k-1);
time_history = time_history(1:k-1);

% Error
e = alpha_history - alpha_desired_history;

% RMSE
RMSE = sqrt(mean(e.^2));

% Max abs error
max_abs_error = max(abs(e));

% Mean |u|
mean_abs_u = mean(abs(u_history));

% Print results
fprintf('\n=== PERFORMANCE METRICS ===\n');
fprintf('RMSE (deg): %.6f\n', RMSE * 180/pi);
fprintf('Max |error| (deg): %.6f\n', max_abs_error * 180/pi);
fprintf('Mean |u|: %.6f\n', mean_abs_u);
fprintf('===========================\n\n');


% PLOTTING
figure('WindowState', 'maximized')

% Angles
subplot(1,2,1)
hold all
plot(time_history, alpha_history(1,:)*180/pi, 'b', 'linewidth', 1)
plot(time_history, alpha_desired_history(1,:)*180/pi, 'r--', 'linewidth', 1)
title('Angles')
xlabel('Time, s')
ylabel('Angle, deg')
legend('Alpha', 'Alpha desired')
grid on

% Angular velocities
subplot(1,2,2)
hold all
plot(time_history, alpha_dot_history(1,:)*180/pi, 'b', 'linewidth', 1)
plot(time_history, alpha_dot_desired_history(1,:)*180/pi, 'r--', 'linewidth', 1)
title('Angular Velocities')
xlabel('Time, s')
ylabel('Angular Velocity, deg/s')
legend('Alpha dot', 'Alpha dot desired')
grid on
box on

% Tracking error (in degrees)
error_history = (alpha_history - alpha_desired_history) * 180/pi;

% RMSE
rmse_alpha = sqrt(mean(error_history.^2));

fprintf('\nRMSE by roll: %.4f degree\n', rmse_alpha);

% Plot error
figure('WindowState', 'maximized')
plot(time_history, error_history, 'k', 'linewidth', 1.2)
xlabel('Time, s')
ylabel('Error, deg')
title('Tracking Error: alpha - alpha_{desired}')
grid on
