% Comparison of closed-loop and open-loop models
clear all
clc

% Parameters
dt = 0.001;
t_final = 20;
n_steps = round(t_final / dt);

a = -0.3;        % ??????????? ?? (???????????)
K = 0.005;

% Initial states (same for both models)
alpha0 = 0;
alpha_dot0 = pi^2/60;

% Time vector
time = (0:n_steps-1) * dt;

% Allocate arrays
alpha_fb = zeros(1, n_steps);      % with feedback
alpha_ol = zeros(1, n_steps);      % open-loop
alpha_des = zeros(1, n_steps);

% Initial conditions
alpha_fb(1) = alpha0;
alpha_ol(1) = alpha0;

alpha_dot_fb = alpha_dot0;
alpha_dot_ol = alpha_dot0;

torque_fb = 0;
torque_ol = 0;

% Simulation loop
for k = 1:n_steps-1
    t = time(k);

    % Desired trajectory
    [alpha_d, alpha_dot_d, ~] = desired_trajectory1(t);
    alpha_des(k) = alpha_d;

    % ===== Model WITH feedback =====
    u_fb = K * (alpha_dot_d + a * (alpha_fb(k) - alpha_d));

    u_wave_fb = linear_model1(alpha_dot_fb);
    torque_fb = u_fb - (u_wave_fb - torque_fb);

    [alpha_fb(k+1), alpha_dot_fb, ~] = ...
        uav_model1(torque_fb, alpha_fb(k), alpha_dot_fb);

    % ===== Model WITHOUT feedback =====
    u_ol = K * alpha_dot_d;

    u_wave_ol = linear_model1(alpha_dot_ol);
    torque_ol = u_ol - (u_wave_ol - torque_ol);

    [alpha_ol(k+1), alpha_dot_ol, ~] = ...
        uav_model1(torque_ol, alpha_ol(k), alpha_dot_ol);
end

% Save last desired value
[alpha_des(end), ~, ~] = desired_trajectory1(time(end));

% Convert to degrees
alpha_fb = alpha_fb * 180/pi;
alpha_ol = alpha_ol * 180/pi;
alpha_des = alpha_des * 180/pi;

% Plotting
figure('WindowState','maximized')

% ---- With feedback ----
subplot(1,2,1)
plot(time, alpha_des, 'r--', 'LineWidth', 1.2); hold on
plot(time, alpha_fb, 'b', 'LineWidth', 1.2)
grid on
xlabel('Time, s')
ylabel('Angle, deg')
title('With feedback')
legend('Desired trajectory', 'Actual trajectory')

% ---- Without feedback ----
subplot(1,2,2)
plot(time, alpha_des, 'r--', 'LineWidth', 1.2); hold on
plot(time, alpha_ol, 'b', 'LineWidth', 1.2)
grid on
xlabel('Time, s')
ylabel('Angle, deg')
title('Without feedback')
legend('Desired trajectory', 'Actual trajectory')

% Errors
e_fb = alpha_fb - alpha_des;   % with feedback
e_ol = alpha_ol - alpha_des;   % without feedback

figure('WindowState','maximized')

plot(time, e_fb, 'b', 'LineWidth', 1.2); hold on
plot(time, e_ol, 'r--', 'LineWidth', 1.2)
grid on
xlabel('Time, s')
ylabel('Error, deg')
title('Tracking error comparison')
legend('With feedback', 'Without feedback')

RMSE_fb = sqrt(mean(e_fb.^2));
RMSE_ol = sqrt(mean(e_ol.^2));

fprintf('\n=== RMSE COMPARISON ===\n');
fprintf('With feedback    RMSE = %.4f deg\n', RMSE_fb);
fprintf('Without feedback RMSE = %.4f deg\n', RMSE_ol);
fprintf('=======================\n');
