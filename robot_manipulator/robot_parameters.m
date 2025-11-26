% Parameters of three-link manipulator
function p = robot_parameters()
    % Geometric parameters [m]
    p.l2 = 0.3;      % Length of link 2
    p.l3 = 0.2;      % Length of link 3  
    p.lg2 = 0.2;    % Distance to COM of link 2
    p.lg3 = 0.3;     % Distance to COM of link 3
    
    % Masses [kg]
    p.m1 = 2.0;      % Mass of link 1
    p.m2 = 1.2;      % Mass of link 2
    p.m3 = 1.0;      % Mass of link 3
    
    % Moments of inertia [kg?m?]
    p.I1 = 0.01;      % Moment of inertia of link 1
    p.I2 = 0.01;     % Moment of inertia of link 2
    p.I3 = 0.01;     % Moment of inertia of link 3
    
    % Motor and friction parameters
    p.D = diag([0.8, 0.6, 0.4]);  % Moment coefficients matrix
    p.F = diag([0.2, 0.08, 0.35]); % Dry friction coefficients
    
    p.J1 = 3.0; p.J2 = 2.5; p.J3 = 0.8;
    p.J_0 = diag([p.J1, p.J2, p.J3]); 
        
    % Gravity acceleration
    p.g = 9.81;      
    
    % Viscous friction coefficients
    p.k1 = 15.5; p.k2 = 15.0; p.k3 = 20.0;
    p.K = diag([1/p.k1, 1/p.k2, 1/p.k3]); 
    
    %Angular velocity and radius
    p.omega = 0.1; p.r = 0.05;
    
    %coordinates of the center of the circle
    p.a = 0.15; p.b = 0; p.c = 0;
    
    %Feedback gain 
    p.alpha = -0.8;
    
    %
    p.dt = 0.001;
    
end