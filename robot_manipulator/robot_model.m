%Model of robot manipulator
function [theta, theta_dot, coordinates] = robot_model(tau, theta_prev, theta_dot_prev)

    p = robot_parameters();
    %{
    M = inertia_matrix(theta_prev, p);    
    h = coriolis_vector(theta_prev, theta_dot_prev, p);
    
    %regularization
    if any(isnan(M(:))) || any(isinf(M(:)))
        M = 1e-6 * eye(3);
    end
    if any(isnan(h(:))) || any(isinf(h(:)))
        h = [1e-6; 1e-6; 1e-6];
    end
    
    friction = p.K * theta_dot_prev + p.F * sign(theta_dot_prev);
    tenzor = p.J_0 + M;
    theta_ddot =  tenzor \ (tau - h - friction);
    %}
    sth2 = 0.3*sin(theta_prev(2))+0.35*sin(theta_prev(3));
    sth3 = 0.15*sin(theta_prev(3));
    
    theta_ddot = [p.dt/p.J1 * (tau(1) - p.k1*theta_dot_prev(1) - 0.2*sign(theta_dot_prev(1)));
                  p.dt/p.J2 * (tau(2) - p.k2*theta_dot_prev(2) - 0.08*sign(theta_dot_prev(2)) + sth2);
                  p.dt/p.J3 * (tau(3) - p.k3*theta_dot_prev(3) - 0.35*sign(theta_dot_prev(3)) + sth3)];
                 
    theta_dot = theta_dot_prev + theta_ddot; 
    theta = theta_prev + theta_dot_prev * p.dt;
    
    [X, Y, Z] = forward_kinematics(theta);
    coordinates = [X, Y, Z];
   
end