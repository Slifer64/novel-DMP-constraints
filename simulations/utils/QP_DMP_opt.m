function [Time, P_data, dP_data, ddP_data] = QP_DMP_opt(gmp, tau, y0, yg, pos_lim, vel_lim, accel_lim)


%% get reference acceleration

n_dof = gmp.numOfDoFs();

gmp.setY0(y0);
gmp.setGoal(yg);

n_data = 200;
x_data = linspace(0,1, n_data);
x_dot = 1/tau;
x_ddot = 0;
ddPd_data = zeros(n_dof, n_data);
for j=1:n_data
   ddPd_data(:,j) = gmp.getYdDDot(x_data(j), x_dot, x_ddot); 
end
Timed = x_data*tau;

%% initialization and training
a_z = 20;
b_z = a_z/4;
N_kernels = 30;
kernels_std_scaling = 1.5;
qpmp = {};
for i=1:n_dof
    qpmp{i} = QPMP(N_kernels, a_z, b_z, kernels_std_scaling);
    qpmp{i}.setDemo(Timed, ddPd_data(i,:), tau, y0(i), yg(i));
end

ydot_0 = zeros(n_dof, 1);

ti = linspace(0,tau, n_data);
t_start = tic;
for i=1:n_dof
    % =======  Position constraints  =======
    pos_constr = [lowerBoundConstr(ti, pos_lim(i,1)); upperBoundConstr(ti, pos_lim(i,2)); struct('t',0, 'value',y0(i), 'type','='); struct('t',tau, 'value',yg(i), 'type','=')];
    % =======  Velocity constraints  =======
    vel_constr = [lowerBoundConstr(ti, vel_lim(i,1)); upperBoundConstr(ti, vel_lim(i,2)); struct('t',0, 'value',0, 'type','='); struct('t',tau, 'value',0, 'type','=')];
    % =======  Acceleration constraints  =======
    accel_constr = [lowerBoundConstr(ti, accel_lim(i,1)); upperBoundConstr(ti, accel_lim(i,2)); struct('t',0, 'value',0, 'type','='); struct('t',tau, 'value',0, 'type','=')];

    [success, exit_status] = qpmp{i}.train(tau, y0(i), ydot_0(i), yg(i), pos_constr, vel_constr, accel_constr);
    
    msg = ['DoF ' num2str(i) ': ' exit_status '\n'];
    if (~success), warning(msg);
    else, fprintf(msg);
    end
    
end
fprintf('===> QP-DMP finished! Elaps time: %f ms\n',toc(t_start)*1000);




%% simulation

Time = [];
P_data = [];
dP_data = [];
ddP_data = [];

p = y0;
p_dot = ydot_0;

dt = 0.002;
for i=1:n_dof
    qpmp{i}.init();
end

while (true)
    
    p_ddot = zeros(n_dof,1);
    for i=1:n_dof
        [t, p_ddot_i] = qpmp{i}.getAccel(p(i), p_dot(i), dt);
        p_ddot(i) = p_ddot_i;
    end
    
    Time = [Time t];
    P_data = [P_data p];
    dP_data = [dP_data p_dot];
    ddP_data = [ddP_data p_ddot];
    
    p = p + p_dot*dt;
    p_dot = p_dot + p_ddot*dt;
    
    if (t >= tau), break; end
    
end


end

%% =========== Utility functions ==============

function constr = upperBoundConstr(ti, bound)

    constr = repmat(struct('t',[], 'value',[], 'type',[]), length(ti), 1);
    for i=1:length(ti)
        constr(i) = struct('t',ti(i), 'value',bound, 'type','<');
    end

end

function constr = lowerBoundConstr(ti, bound)

    constr = repmat(struct('t',[], 'value',[], 'type',[]), length(ti), 1);
    for i=1:length(ti)
        constr(i) = struct('t',ti(i), 'value',bound, 'type','>');
    end

end

function constr = thresholdConstr(ti, thres)
    
    constr = [upperBoundConstr(ti, thres); lowerBoundConstr(ti, -thres)];

end
