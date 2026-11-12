clear
clear function_correction
clc
rng(2); 
set(0, 'DefaultFigureColor', 'white');
set(0, 'DefaultAxesColor', 'white');
set(0, 'DefaultFigureWindowState', 'maximized');


% Simulation parameters
dt = 0.001;
t_final = 20;
t = 0;

% Initial state
[alpha, alpha_dot, alpha_ddot] = desired_trajectory(0);

n_steps = round(t_final/dt);

% Noise parameters
angle_noise_std = deg2rad(2.6);
rate_noise_std = deg2rad(2.8);
disturbance_std = 0.04;

% Memory allocation
alpha_history = zeros(1,n_steps);
alpha_dot_history = zeros(1,n_steps);
alpha_ddot_history = zeros(1,n_steps);

alpha_d_history = zeros(1,n_steps);
alpha_dot_d_history = zeros(1,n_steps);
alpha_ddot_d_history = zeros(1,n_steps);

torque_history = zeros(1,n_steps);
time_history = zeros(1,n_steps);

k = 1;
u_delta = 0;

while t < t_final

    % Desired trajectory
    [alpha_d, alpha_dot_d, alpha_ddot_d] = desired_trajectory(t);
    
    % Sensor noise
    alpha = alpha + angle_noise_std*randn;
    alpha_dot = alpha_dot + rate_noise_std*randn;

    % Control
    u_bar = controller(alpha,alpha_dot,t);
    
    torque = u_bar - u_delta;
    
    % External disturbance
    disturbance = disturbance_std*randn;
    
    torque = torque + disturbance;

    % UAV dynamics
    [alpha_new,alpha_dot_new, alpha_ddot_new] = uav_model(torque,alpha,alpha_dot);
    
    % Save data
    alpha_history(k) = alpha_new;
    alpha_dot_history(k) = alpha_dot_new;
    alpha_ddot_history(k) = alpha_ddot_new;


    alpha_d_history(k) = alpha_d;
    alpha_dot_d_history(k) = alpha_dot_d;
    alpha_ddot_d_history(k) = alpha_ddot_d;

    torque_history(k) = torque;
    time_history(k) = t;

    % Update state
    alpha = alpha_new;
    alpha_dot = alpha_dot_new;
    
    % inner loop
    u_wave = inverse_linear_model(alpha_dot);
    u_delta = u_wave;
    

    t = t + dt;
    k = k + 1;

end


error = alpha_history - alpha_d_history;

RMSE = sqrt(mean(error.^2));
%RMSE = norm(error,2);

fprintf("RMSE = %.4f deg\n",RMSE*180/pi)

%{
fontName = 'Arial Unicode MS';   
fontSizeAxis = 12;
fontSizeLegend = 12;
lineWidth = 2;

set(groot,'DefaultAxesFontName',fontName);
set(groot,'DefaultTextFontName',fontName);
set(groot,'DefaultLegendFontName',fontName);


figure

subplot(3,1,1)

plot(time_history,alpha_history*180/pi,'b','LineWidth',lineWidth)
hold on
plot(time_history,alpha_d_history*180/pi,'r--','LineWidth',lineWidth)

xlabel('Time, s','FontSize',fontSizeAxis,'FontName',fontName)
ylabel('Angle, deg','FontSize',fontSizeAxis,'FontName',fontName)

legend({'Actual angle','Desired angle'}, ...
       'FontSize',fontSizeLegend, ...
       'Location','best', ...
       'FontName',fontName)

set(gca,'FontSize',fontSizeAxis,'FontName',fontName)

grid on
box on


subplot(3,1,2)

plot(time_history,alpha_dot_history*180/pi,'b','LineWidth',lineWidth)
hold on
plot(time_history,alpha_dot_d_history*180/pi,'r--','LineWidth',lineWidth)

xlabel('Time, s','FontSize',fontSizeAxis,'FontName',fontName)
ylabel('Angular velocity, deg/s','FontSize',fontSizeAxis,'FontName',fontName)

legend({'Actual angular velocity','Desired angular velocity'}, ...
       'FontSize',fontSizeLegend, ...
       'Location','best', ...
       'FontName',fontName)

set(gca,'FontSize',fontSizeAxis,'FontName',fontName)

grid on
box on

subplot(3,1,3)

plot(time_history,alpha_ddot_history*180/pi,'b','LineWidth',lineWidth)
hold on
plot(time_history,alpha_ddot_d_history*180/pi,'r--','LineWidth',lineWidth)

xlabel('Time, s','FontSize',fontSizeAxis,'FontName',fontName)
ylabel('Angular velocity, deg/s','FontSize',fontSizeAxis,'FontName',fontName)

legend({'Actual angular velocity','Desired angular velocity'}, ...
       'FontSize',fontSizeLegend, ...
       'Location','best', ...
       'FontName',fontName)

set(gca,'FontSize',fontSizeAxis,'FontName',fontName)

grid on
box on



figure

plot(time_history,error*180/pi,'k','LineWidth',lineWidth)

xlabel('Time, s','FontSize',fontSizeAxis,'FontName',fontName)
ylabel('Error, deg','FontSize',fontSizeAxis,'FontName',fontName)

set(gca,'FontSize',fontSizeAxis,'FontName',fontName)

grid on
box on
%}