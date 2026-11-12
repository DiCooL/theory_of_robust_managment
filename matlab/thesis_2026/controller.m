function torque = controller(alpha,alpha_dot,t)

[alpha_d,alpha_dot_d,alpha_ddot_d] = desired_trajectory(t);

mu = 2.5e-3;

%a<0
%-(a^2)/4 < b < 0
%a = -1671.49, -202.00,
%b = -684706.60, -10000.00
a = -202.00;
b = -10000.00;

torque = -(0.3*sin(alpha) - 2*mu*alpha_dot) + (a*alpha_dot + b*(alpha - alpha_d))*0.02;

end