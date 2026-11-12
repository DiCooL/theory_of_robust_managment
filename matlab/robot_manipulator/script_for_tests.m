clear all
clc

p = robot_parameters();
%initial states
t = 0; 
theta = [0; pi/2; pi]; 
theta_dot = [0; pi/4; pi/3];

repeatTime = 2;
t_final = repeatTime*(2 * pi / p.omega);
n_steps = round(t_final / p.dt);

% Arrays for storing results
theta_history = zeros(3, n_steps);
theta_dot_history = zeros(3, n_steps);
coordinates_history = zeros(3, n_steps);
time_history = zeros(1, n_steps);

f = waitbar(0, 'Simulation running...');

k = 1;
tau = [0; 0; 0];
while t < t_final
    
    % Control signals
    u_fil = 1 * tau; % filter corrector
    u_wave = linear_model(theta_dot);
    u_del = u_wave - u_fil;
    
    u = trajectory_control(t, theta);
    tau = u - u_del;
    
    % Dynamics
    [theta_new, theta_dot_new, coordinates] = robot_model(tau, theta, theta_dot);

    % Save results
    theta_history(:, k) = theta_new;
    theta_dot_history(:, k) = theta_dot_new;
    coordinates_history(:, k) = coordinates;
    time_history(k) = t;
    
    % Update state
    theta = theta_new; 
    theta_dot = theta_dot_new;
    t = t + p.dt;
    k = k + 1;
    
    % Progress
    waitbar(t/t_final, f, sprintf('Progress: %.1f%%', t/t_final*100));
end

close(f);

% Trim arrays to actual number of steps
theta_history = theta_history(:, 1:k-1);
theta_dot_history = theta_dot_history(:, 1:k-1);
coordinates_history = coordinates_history(:, 1:k-1);
time_history = time_history(1:k-1);

% Calculate desired trajectory for plots
XYZ_fun = @(t) [...
    p.a-p.r*cos(p.omega*t);...
    p.b;...
    p.c+p.r*sin(p.omega*t)];
XYZ_desired = zeros(3, length(time_history));
for i = 1:length(time_history)
    XYZ_desired(:, i) = XYZ_fun(time_history(i));
end

% PLOTTING
figure

% Trajectory in XZ plane (circle)
subplot(1,2,1)
hold all
plot(XYZ_desired(1,:), XYZ_desired(3,:), 'r', 'linewidth', 2)
plot(coordinates_history(1,:), coordinates_history(3,:), 'b', 'linewidth', 1)
title('Trajectory in XZ Plane')
xlabel('X, m')
ylabel('Z, m')
legend('Desired', 'Actual')
axis equal
grid on
box on

% Total error norm
subplot(1,2,2)
error_norm = sqrt((XYZ_desired(1,:) - coordinates_history(1,:)).^2 + ...
                  (XYZ_desired(2,:) - coordinates_history(2,:)).^2 + ...
                  (XYZ_desired(3,:) - coordinates_history(3,:)).^2);
plot(time_history, error_norm, 'k', 'linewidth', 1)
title('Error Norm')
xlabel('Time, s')
ylabel('||e||, m')
grid on
box on

figure

% X coordinate
subplot(2,3,1)
hold all
plot(time_history, XYZ_desired(1,:), 'r', 'linewidth', 2)
plot(time_history, coordinates_history(1,:), 'b', 'linewidth', 1)
title('X Coordinate')
xlabel('Time, s')
ylabel('X, m')
legend('Desired', 'Actual')
grid on
box on

% X error
subplot(2,3,4)
plot(time_history, XYZ_desired(1,:) - coordinates_history(1,:), 'k', 'linewidth', 1)
title('X Error')
xlabel('Time, s')
ylabel('e_X, m')
grid on
box on

% Y coordinate  
subplot(2,3,2)
hold all
plot(time_history, XYZ_desired(2,:), 'r', 'linewidth', 2)
plot(time_history, coordinates_history(2,:), 'b', 'linewidth', 1)
title('Y Coordinate')
xlabel('Time, s')
ylabel('Y, m')
legend('Desired', 'Actual')
grid on
box on

% Y error
subplot(2,3,5)
plot(time_history, XYZ_desired(2,:) - coordinates_history(2,:), 'k', 'linewidth', 1)
title('Y Error')
xlabel('Time, s')
ylabel('e_Y, m')
grid on
box on

% Z coordinate
subplot(2,3,3)
hold all
plot(time_history, XYZ_desired(3,:), 'r', 'linewidth', 2)
plot(time_history, coordinates_history(3,:), 'b', 'linewidth', 1)
title('Z Coordinate')
xlabel('Time, s')
ylabel('Z, m')
legend('Desired', 'Actual')
grid on
box on

% Z error
subplot(2,3,6)
plot(time_history, XYZ_desired(3,:) - coordinates_history(3,:), 'k', 'linewidth', 1)
title('Z Error')
xlabel('Time, s')
ylabel('e_Z, m')
grid on
box on

fprintf('Simulation completed!\n');
fprintf('Maximum error: %.4f m\n', max(error_norm));
fprintf('Average error: %.4f m\n', mean(error_norm));
