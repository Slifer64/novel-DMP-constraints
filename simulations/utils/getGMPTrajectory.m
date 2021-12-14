function [Time, P_data, dP_data, ddP_data] = getGMPTrajectory(gmp, tau, y0, yg0, yg, t_g)

    if (nargin < 5), yg = yg0; end
    if (nargin < 6), t_g = inf; end
       
    Time = [];
    P_data = [];
    dP_data = [];
    ddP_data = [];

    p = y0;
    p_dot = zeros(size(p));
    p_ddot = zeros(size(p));
    
    gmp.setY0(y0);
    gmp.setGoal(yg0);

    t = 0;
    dt = 0.002;
    
    x = 0;
    
    p_ddot_prev = zeros(size(p));

    while (true)
        
        if (t >= t_g)
           t_g = inf;
           gmp.setGoal(yg);
        end
        
        x_dot = 1/tau;
        
        % if (x >= 1), x_dot = 0; end
        
        p_ref = gmp.getYd(x);
        p_ref_dot = gmp.getYdDot(x, x_dot);
        p_ref_ddot = gmp.getYdDDot(x, x_dot, 0);
        
%         p_ddot = 300*(p_ref - p) + 80*(p_ref_dot - p_dot) + p_ref_ddot;
        
        p = p_ref;
        p_dot = p_ref_dot;
        p_ddot = p_ref_ddot;

        Time = [Time t];
        P_data = [P_data p];
        dP_data = [dP_data p_dot];
        ddP_data = [ddP_data p_ddot];

        t = t + dt;
        x = x + x_dot*dt;
        p = p + p_dot*dt;
        p_dot = p_dot + p_ddot*dt;

        if (x >= 1.0) % && norm(p_dot)<5e-3 && norm(p_ddot)<1e-2)
            break; 
        end

    end

end
