% Linear model of UAV
function u_wave = linear_model1(alpha_dot)
    
    mu = 2.5e-3;
       
    u_wave = 2 * mu * alpha_dot;

end