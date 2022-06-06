clc;
close all;
clear;

% rng(0);


load('data/obs_avoid_data', 'sd_data', 'Pd_data')

n_dof = size(Pd_data, 1);

ellipsoid = {};
ellipsoid = [ellipsoid {struct('Sigma',getEllipseSigma(45, 0.15, 0.08), 'c',[-0.1; 0.3])}];
% ellipsoid = [ellipsoid {struct('Sigma',getEllipseSigma(-45, 0.1, 0.06), 'c',[0.18; 0.61])}];
ellipsoid_colors = {[1.0, 0.4, 0], [0.4, 0.2, 0]};

for i=1:length(ellipsoid)  
    E_p{i} = drawElipsoid2D(ellipsoid{i}.Sigma, ellipsoid{i}.c);
end


addpath('utils/');
import_gmp_lib();
import_io_lib();

gmp = GMP(2, 30, 1.5);
train_err = gmp.train('LS', sd_data, Pd_data)
for j=1:length(sd_data), Pd_data(:,j) = gmp.getYd(sd_data(j)); end

% P_data = [];
% for i=1:length(sd_data)
%     P_data = [P_data gmp.getYd(sd_data(i))];
% end
% figure; hold on;
% plot(P_data(1,:), P_data(2,:));
% plot(Pd_data(1,:), Pd_data(2,:));

%% -------- Limits ---------
pos_lim = [
    [ -0.4 , 0.4];
    [ -0.05 , 0.8];
  ];

vel_lim = repmat([ -3.8 , 3.8 ], n_dof, 1);

accel_lim = repmat([ -10 , 10 ], n_dof, 1);

% pos_lim = pos_lim + repmat([-1 1], n_dof, 1); % to augment the limits

%% --------- Optimization objective ----------
opt_pos = 1;
opt_vel = 0;

%% -------- Initial/Final states --------
y0 = Pd_data(:, 1);
yg = Pd_data(:, end);
Tf = 6;
tau = Tf;
dt = 0.005;

gmp.setY0(y0);
gmp.setGoal(yg);


%% =============  Run  ==================

fig = figure;
fig.Position(3:4) = [1112 844];
ax = axes();
hold(ax, 'on');
plot(Pd_data(1, :), Pd_data(2, :), 'LineWidth',2);
plot(Pd_data(1, 1), Pd_data(2, 1), 'LineWidth',2, 'Marker','o', 'Color',[0 0.8 0], 'MarkerSize',14);
plot(Pd_data(1, end), Pd_data(2, end), 'LineWidth',2, 'Marker','x', 'Color',[0.8 0 0], 'MarkerSize',14);
for i=1:length(ellipsoid)
    plot(E_p{i}(1,:), E_p{i}(2,:), 'LineWidth',2, 'Color',ellipsoid_colors{i});
    plot(ellipsoid{i}.c(1), ellipsoid{i}.c(2), 'LineWidth',2, 'Marker','x', 'LineStyle','None', 'Markersize',14, 'Color',ellipsoid_colors{i});
end
axis equal

data = {};
opt_type = 'p';
if (opt_vel), opt_type = 'v'; end

%% --------- Simple Execution -----------
gmp.setScaleMethod(TrajScale_Prop(n_dof));
[Time, P_data, dP_data, ddP_data] = getGMPTrajectory(gmp, Tf, y0, yg);
data{length(data)+1} = ...
    struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle',':', ...
        'color','blue', 'legend',['$' 'DMP' '$'], 'plot3D',true, 'plot2D',true); 

    
%% ---------- GMP-MPC optimization ------------
[Time, P_data, dP_data, ddP_data] = gmpMpcOpt(gmp, dt, Tf, y0, yg, pos_lim, vel_lim, accel_lim, ellipsoid, opt_pos, opt_vel, ax);

data{length(data)+1} = ...
    struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','-', ...
    'color',[0.72 0.27 1], 'legend','$\overline{DMP}^*$', 'plot3D',true, 'plot2D',true);


%% --------- Plot results ------------
figure; hold on;
plot(Pd_data(1, :), Pd_data(2, :), 'LineWidth',2);
plot(Pd_data(1, 1), Pd_data(2, 1), 'LineWidth',2, 'Marker','o', 'Color',[0 0.8 0], 'MarkerSize',14);
plot(Pd_data(1, end), Pd_data(2, end), 'LineWidth',2, 'Marker','x', 'Color',[0.8 0 0], 'MarkerSize',14);
plot(P_data(1, :), P_data(2, :), 'LineWidth',2, 'Color','magenta');
for i=1:length(ellipsoid)
    plot(E_p{i}(1,:), E_p{i}(2,:), 'LineWidth',2, 'Color',ellipsoid_colors{i});
    plot(ellipsoid{i}.c(1), ellipsoid{i}.c(2), 'LineWidth',2, 'Marker','x', 'LineStyle','None', 'Markersize',14, 'Color',ellipsoid_colors{i});
end
axis equal


title_ = {'x coordinate', 'y coordinate', 'z coordinate'};
label_font = 17;
ax_fontsize = 14;

ind = [1 2]; % choose DoFs to plot, e.g. [1 2 3] for [x y z]
for k=1:length(ind)
    
    i = ind(k);
    
    fig = figure;
    fig.Position(3:4) = [842 1110];

    ax = subplot(3,1,1);
    hold on;
    % plot position trajectory
    legend_ = {};
    for k=1:length(data)
        if (~data{k}.plot2D), continue; end
        plot(data{k}.Time, data{k}.Pos(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color);
        legend_ = [legend_ data{k}.legend];
    end
    axis tight;
    % plot start and final positions
    plot(0, y0(i), 'LineWidth', 4, 'LineStyle','none', 'Color','green','Marker','o', 'MarkerSize',10);
    plot(tau, yg(i), 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10);
%     plot(tau, ygd(i), 'LineWidth', 4, 'LineStyle','none', 'Color','magenta','Marker','x', 'MarkerSize',10);
    % plot bounds
    plot(ax.XLim, [pos_lim(i,1) pos_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
    plot(ax.XLim, [pos_lim(i,2) pos_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
    % labels, title ...
    ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',label_font);
%     title(title_{i}, 'interpreter','latex', 'fontsize',18);
    legend(legend_, 'interpreter','latex', 'fontsize',17, 'Position',[0.2330 0.9345 0.5520 0.0294], 'Orientation', 'horizontal');
    ax.FontSize = ax_fontsize;
    hold off;

    ax = subplot(3,1,2);
    hold on;
    for k=1:length(data)
        if (~data{k}.plot2D), continue; end
        plot(data{k}.Time, data{k}.Vel(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color);
    end
    axis tight;
    % plot bounds
    plot(ax.XLim, [vel_lim(i,1) vel_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
    plot(ax.XLim, [vel_lim(i,2) vel_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
    ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',label_font);
    ax.FontSize = ax_fontsize;
    hold off;

    ax = subplot(3,1,3);
    hold on;
    for k=1:length(data)
        if (~data{k}.plot2D), continue; end
        plot(data{k}.Time, data{k}.Accel(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color);
    end
    axis tight;
    % plot bounds
    plot(ax.XLim, [accel_lim(i,1) accel_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
    plot(ax.XLim, [accel_lim(i,2) accel_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
    ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',label_font);
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',label_font);
    ax.FontSize = ax_fontsize;
    hold off;

end


%% ===================================
%% ===================================

function [Time, P_data, dP_data, ddP_data] = gmpMpcOpt(gmp0, dt, Tf, y0, yg, pos_lim, vel_lim, accel_lim, obstacles, opt_pos, opt_vel, ax)
      
    gmp = gmp0.deepCopy();
   
    n_dof = length(y0);

    O_ndof = zeros(n_dof,1);

    t = 0;
    can_sys = CanonicalSystem(Tf, 30);
    y = y0;
    y_dot = O_ndof;
    y_ddot = O_ndof;
    
    N_horizon = 10;
    pred_time_step = 0.1;
    N_kernels = 30;
    kernels_std_scaling = 1.5;
    
    final_state_err_tol = 1e-1*[1e-3; 1e-2; 1e-1];
    
    slack_gains = [1e5 100 1];
    slack_limits = [2e-2, 0.2, 1.0];
    
    time_limit = 0; %2e-3;
    max_iter = 12000;
    abs_tol = 1e-3;
    rel_tol = 1e-5;
        
    %% --------  GMP - MPC  --------
    gmp_mpc = GMP_MPC(gmp, N_horizon, pred_time_step, N_kernels, kernels_std_scaling, slack_gains);
    
    gmp_mpc.ax = ax;
    
    gmp_mpc.settings.max_iter = max_iter;
    gmp_mpc.settings.time_limit = time_limit;
    gmp_mpc.settings.abs_tol = abs_tol;
    gmp_mpc.settings.rel_tol = rel_tol;
    
    gmp_mpc.setObjCostGains(opt_pos, opt_vel);
    
    gmp_mpc.setPosLimits(pos_lim(:,1), pos_lim(:,2));
    gmp_mpc.setVelLimits(vel_lim(:,1), vel_lim(:,2));
    gmp_mpc.setAccelLimits(accel_lim(:,1), accel_lim(:,2));
    
    gmp_mpc.setPosSlackLimit(slack_limits(1));
    gmp_mpc.setVelSlackLimit(slack_limits(2));
    gmp_mpc.setAccelSlackLimit(slack_limits(3));

    gmp_mpc.setInitialState(y, y_dot, y_ddot, 0, can_sys.sd_dot, 0);
    gmp_mpc.setFinalState(yg, O_ndof, O_ndof, 1, can_sys.sd_dot, 0, final_state_err_tol);
    
    can_sys_fun = @(s, s_dot) [s_dot; can_sys.getPhaseDDot(s, s_dot)];
    gmp_mpc.setCanonicalSystemFunction(can_sys_fun);
    
    for i=1:length(obstacles)
       gmp_mpc.addEllipsoidObstacle(obstacles{i}.c, obstacles{i}.Sigma)
    end
    
    gmp.setScaleMethod(TrajScale_Prop(n_dof));
    gmp.setY0(y0);
    gmp.setGoal(yg);
    
    text_prog = ProgressText(40);
    text_prog.init();
    
    t_start = tic;
    
    opt_fail = false;
    on_opt_fail_exit = true;
    
    Time = [];
    P_data = [];
    dP_data = [];
    ddP_data = [];
    pos_slack_data = [];
    vel_slack_data = [];
    accel_slack_data = [];

    
    %% -------  Simulation loop  --------
    while (true)
        
        %% update final state
        gmp_mpc.setFinalState(yg, O_ndof, O_ndof, 1, can_sys.sd_dot, 0, final_state_err_tol);
            
        %% Stopping criteria
        if (can_sys.s >= 1), break; end
        
        % display progress
        if (can_sys.s <= 1), text_prog.update(100*can_sys.s); end
          
        gmp_mpc.P_data = P_data;
        
        %% optimization
        if (~opt_fail)
            
            % sovle
            sol = gmp_mpc.solve(can_sys.s, can_sys.s_dot);

            % check exit status
            if (sol.exit_flag)
                warning(sol.exit_msg);
                t
                s = can_sys.s
                text_prog.printInNewLine();
                if (sol.exit_flag < 0)
                    opt_fail = true;
                    if (on_opt_fail_exit), break; end 
                end
            end
            
        end
        
        %% get optimal trajectory
        s = can_sys.s;
        s_dot = can_sys.s_dot;
        s_ddot = can_sys.getPhaseDDot(s, s_dot);
        
        y = gmp_mpc.getYd(s); %sol.y;
        y_dot = gmp_mpc.getYdDot(s, s_dot); %sol.y_dot;
        y_ddot = gmp_mpc.getYdDDot(s, s_dot, s_ddot); %sol.y_ddot;
        
        %% update initial state (optionally, if say a perturbation occurs)
        % gmp_mpc.setInitialState(y, y_dot, y_ddot, s, s_dot, s_ddot);

        %% Log data
        Time = [Time t];
        P_data = [P_data y];
        dP_data = [dP_data y_dot];
        ddP_data = [ddP_data y_ddot];

        if (~opt_fail)
            pos_slack_data = [pos_slack_data sol.pos_slack];
            vel_slack_data = [vel_slack_data sol.vel_slack];
            accel_slack_data = [accel_slack_data sol.accel_slack];
        end
        
        %% Numerical integration
        can_sys.integrate(t, t+dt);
        t = t + dt;
    
    end

    if (s >= 1), text_prog.update(100); end

    if (~isempty(Time))
        
        plotSlackVariables(Time, pos_slack_data, vel_slack_data, accel_slack_data, slack_limits);
        
        target_err = norm(P_data(:,end)-yg)
        vel_err = norm(dP_data(:,end))
        accel_err = norm(ddP_data(:,end))
        
        max_slack_violations = [max(abs(pos_slack_data(:))), max(abs(vel_slack_data(:))), max(abs(accel_slack_data(:)))]
    end
    
    fprintf('\n');
    fprintf('===> GMP-MPC optimization finished! Elaps time: %f ms\n',toc(t_start)*1000);

end


function p = drawElipsoid2D(Sigma, c)

    theta = linspace(0, 2*pi, 200);
    L = chol(Sigma, 'lower'); % sqrtm(Sigma);
    p = c + L*[cos(theta); sin(theta)];

end

function Sigma = getEllipseSigma(angle, lambda_x, lambda_y)

    theta = angle * pi / 180;
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    Lambda_2 = diag([lambda_x, lambda_y]);
    Sigma = R * Lambda_2.^2 * R';

end
