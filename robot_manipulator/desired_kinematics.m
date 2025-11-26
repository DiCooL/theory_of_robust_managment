%Desired trajectory
function [X, Y, Z] = desired_kinematics(t)

    p = robot_parameters();

    X = p.a - p.r*cos(p.omega*t);
    Y = p.b;
    Z = p.c + p.r*sin(p.omega*t);
  
end