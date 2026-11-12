function u_fil = function_correction(u)

    T = 15;
    global dt

    persistent y_prev

    if isempty(y_prev)
        y_prev = 0;
    end

    u_fil = y_prev + dt * T * (u - y_prev);

    y_prev = u_fil;

end