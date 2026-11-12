%
function [alpha_d, alpha_dot_d, alpha_ddot_d] = desired_trajectory(t)

A = pi/6;          % Amplitude
bias = pi/6;       % Bias
omega = pi/10;     % Frequency (rad/sec)
phase = -pi/2;     % Phase (rad)

% Position
alpha_d = A * sin(omega * t + phase) + bias;

% Velocity (first derivative)
alpha_dot_d = A * omega * cos(omega * t + phase);

% Acceleration (second derivative)
alpha_ddot_d = -A * omega^2 * sin(omega * t + phase);

end
%}

%{
function [alpha_d, alpha_dot_d, alpha_ddot_d] = desired_trajectory(t)

A = pi/6;      % Amplitude
bias = 0;      % Bias
k = 0.8;       % Speed parameter

% Position
alpha_d = A * atan(k * t) + bias;

% Velocity
alpha_dot_d = A * k / (1 + (k*t)^2);

% Acceleration
alpha_ddot_d = -2 * A * k^3 * t / (1 + (k*t)^2)^2;

end
%}

%{
function [alpha_d, alpha_dot_d, alpha_ddot_d] = desired_trajectory(t)

A = pi/6;      
bias = -pi/12;      
k = 0.2;       

alpha_d = (A * (atan(k * t) + pi/2) / pi) + bias;

alpha_dot_d = (A * k) / (pi * (1 + (k*t)^2));

alpha_ddot_d = -(2 * A * k^3 * t) / (pi * (1 + (k*t)^2)^2);

end
%}

%{
function [alpha_d, alpha_dot_d, alpha_ddot_d] = desired_trajectory(t)

A = pi/6;      % Amplitude
bias = 0;      % Bias
k = 0.8;       % Speed parameter

% Position
alpha_d = (t>2/3) - (t>4/3); %for a = -1000, b = - 240000

% Velocity
alpha_dot_d = 0;

% Acceleration
alpha_ddot_d = 0;

end
%}
%{
function [alpha_d, alpha_dot_d, alpha_ddot_d] = desired_trajectory(t)

A = pi/6;      % Amplitude
bias = 0;      % Bias
k = 0.8;       % Speed parameter

% Position
alpha_d = cos(t)-1; %for a = -1000, b = - 240000

% Velocity
alpha_dot_d = -sin(t);

% Acceleration
alpha_ddot_d = -cos(t);

end
%}