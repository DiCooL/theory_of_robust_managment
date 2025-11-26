%Linear model of robot manipulator
function u = linear_model(theta_dot)

    p = robot_parameters();
    
    u = p.K * theta_dot; %[3 x 3] * [3 x 1]
    
end