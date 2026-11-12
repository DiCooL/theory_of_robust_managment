classdef CorrectionFilter < handle
    properties
        y_prev
        dt
    end
    
    methods
        function obj = CorrectionFilter(dt)
            obj.dt = dt;
            obj.reset();
        end
        
        function u_fil = update(obj, u)
            % y[n+1] = y[n] + dt*(u[n] - 3*y[n])
            u_fil = obj.y_prev + obj.dt * (u - 3*obj.y_prev);
            obj.y_prev = u_fil;
        end
        
        function reset(obj)
            obj.y_prev = 0;
        end
    end
end