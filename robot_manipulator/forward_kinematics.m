function [X, Y, Z] = forward_kinematics(theta)
    
    p_params = robot_parameters();
    
    % theta = [theta1, theta2, theta3]
    theta1 = theta(1);
    theta2 = theta(2); 
    theta3 = theta(3);
    
    % radius 
    p_val = p_params.l2 * sin(theta2) + p_params.l3 * sin(theta2 + theta3);
    
    % coordinate of end robot
    X = p_val * cos(theta1);
    Y = p_val * sin(theta1);
    Z = p_params.l2 * cos(theta2) + p_params.l3 * cos(theta2 + theta3);
    
    
end