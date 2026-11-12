% Function correction of uav
function u_cap = function_correction1(t, u)

    [~, alpha_dot_desired, ~] = desired_trajectory1(t);
    
    
    

    u_cap = (u - 0.005 * alpha_dot_desired) + 1;
        
end