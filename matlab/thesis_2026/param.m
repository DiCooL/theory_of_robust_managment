    clear; clc;

    mu = 2.5e-3;

    T = 15;         % ???????? ???????
    Tf = 10;        % ????? ?????????????
    dt = 0.001;
    t = 0:dt:Tf;

    omega_vec = 90:10:100;
    zeta_vec  = 1.00:0.01:1.02; %1.01


    J_best = inf;

    for i = 1:length(omega_vec)
        for j = 1:length(zeta_vec)
            clear function_correction

            omega = omega_vec(i);
            zeta  = zeta_vec(j);

            % ??????? ? a, b
            a = -2*zeta*omega;
            b = -omega^2;

            if ~(a < 0 && b < 0 && b > -(a^2)/4)
                continue
            end

            % ????????? ???????
            alpha = 0;
            alpha_dot = 0;
            u_fil = 0;
            delta_u  = 0;

            e2_sum = 0;

            for k = 1:length(t)

                tk = t(k);

                % ???????? ??????????
                A = pi/6;          
                bias = pi/6;       
                omg = pi/10;     
                phase = -pi/2;

                alpha_d = A * sin(omg * tk + phase) + bias;
                alpha_d_dot = A * omg * cos(omg * tk + phase);
                alpha_d_ddot = -A * omg^2 * sin(omg * tk + phase);


                % ??????
                e = alpha - alpha_d;

                % ?????????
                u_bar = -(0.3*sin(alpha) - 2*mu*alpha_dot) + (a*alpha_dot + b*(alpha - alpha_d))*0.02;

                % ???????? ??????????
                u = u_bar - delta_u;

                % ???????? ???????
                alpha_ddot = (u + 0.3*sin(alpha) - 2*mu*alpha_dot)/0.02;

                % ??????????????
                alpha_dot = alpha_dot + alpha_ddot*dt;
                alpha = alpha + alpha_dot*dt;

                % 
                e2_sum = e2_sum + e^2;

                % ???????? ??????
                u_wave = 2 * mu * alpha_dot;

                % ??????
                u_fil = function_correction(u);

                % ?????????
                delta_u = u_wave - u_fil;

            end

            % ?????????? ????????
            J_val = e2_sum / length(t);

            % ????? ????????
            if J_val < J_best
                J_best = J_val;
                best_omega = omega;
                best_zeta = zeta;
                best_a = a;
                best_b = b;
            end

        end
    end

    fprintf('??????????? ?????????:\n');
    fprintf('omega_n = %.2f\n', best_omega);
    fprintf('zeta = %.2f\n', best_zeta);
    fprintf('a = %.2f\n', best_a);
    fprintf('b = %.2f\n', best_b);
    fprintf('J = %.10f\n', J_best);
    %{
    FOR SIN & Tf = 10
    omega_n = 828.27
    zeta = 1.01
    a = -1673.11
    b = -686031.19
    J = 0.0000000859
    %}