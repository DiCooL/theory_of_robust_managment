function JJ = jacobian_matrix(theta)
    p = robot_parameters();
    theta1 = theta(1); theta2 = theta(2); theta3 = theta(3);
    
    p_val = p.l2*sin(theta2) + p.l3*sin(theta2 + theta3);
    Z_val = p.l2*cos(theta2) + p.l3*cos(theta2 + theta3);
    
    JJ = [-p_val*sin(theta1),  p.l2*cos(theta1)*Z_val,  p.l3*cos(theta1)*cos(theta2+theta3);
           p_val*cos(theta1),  p.l2*sin(theta1)*Z_val,  p.l3*sin(theta1)*cos(theta2+theta3);
           0,                 -p.l2*sin(theta2) - p.l3*sin(theta2+theta3), -p.l3*sin(theta2+theta3)];
       
end