clc;
close all;
clear;

addpath('utils/');
import_gmp_lib();
import_io_lib();


%% -------- load GMP model --------
gmp = GMP();
gmp_.read(gmp, 'data/model.bin');

n_dof = gmp.numOfDoFs();

%% -------- Limits ---------
pos_lim = [
    [ -0.2 , 0.65];
    [ -10.1 , 10.3];
    [ -10.05 , 10.6];
  ];

vel_lim = repmat([ -0.4 , 0.4 ], n_dof, 1);

accel_lim = repmat([ -0.8 , 0.8 ], n_dof, 1);


%% --------- Optimization objective ----------
opt_pos = 1;
opt_vel = 0;

%% -------- Initial/Final states ---------

ygd = [0.26, -0.12, 0.32]';   % demo target
yg = [ 0.55, 0.0817, 0.46 ]'; % execution target
tau = 3.5; % execution time duration

y0 = [-0.0956; -0.443; 0.196];

gmp.setY0(y0);
gmp.setGoal(yg);


%% =============  Run  ==================

data = {};
opt_type = 'p';
if (opt_vel), opt_type = 'v'; end


%% --------- Proportional scaling -----------
gmp.setScaleMethod(TrajScale_Prop(n_dof));
[Time, P_data, dP_data, ddP_data] = getGMPTrajectory(gmp, tau, y0, yg);
data{length(data)+1} = ...
    struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle',':', ...
        'color','blue', 'legend',['$' 'DMP' '$'], 'plot3D',true, 'plot2D',true); 
        
    
%% ---------- GMP-MPC optimization ------------

slack_limits = [2e-2, 0.05, 0.5];

% with slacks
disp('Test 1/3: with relaxation variables');
[Time, P_data, dP_data, ddP_data, slack_data] = gmpMpcOpt(gmp, tau, y0, yg, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel, true, slack_limits);

data{length(data)+1} = ...
    struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','-', ...
    'color',[0.72 0.27 1], 'legend',['relax on'], 'plot3D',true, 'plot2D',true);

% without slacks
disp('Test 2/3: without relaxation variables');
[Time, P_data, dP_data, ddP_data] = gmpMpcOpt(gmp, tau, y0, yg, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel, false, []);

data{length(data)+1} = ...
    struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','-', ...
    'color',[0 0.7 0], 'legend',['relax off'], 'plot3D',true, 'plot2D',true);

% without slacks and augment actual limits by the slack limits
disp('Test 3/3: without relaxation variables and augemented limits (by the relaxation margin)');
[Time, P_data, dP_data, ddP_data] = gmpMpcOpt(gmp, tau, y0, yg, ...
                            pos_lim + repmat(slack_limits(1)*[-1 1],n_dof,1), ...
                            vel_lim + repmat(slack_limits(2)*[-1 1],n_dof,1), ...
                            accel_lim + repmat(slack_limits(3)*[-1 1],n_dof,1), ...
                            opt_pos, opt_vel, false, []);

data{length(data)+1} = ...
    struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle',':', ...
    'color',[0.85 0.33 0.1], 'legend',['relax off + augment limits'], 'plot3D',true, 'plot2D',true);

%% ======= Plot results =========

i = 1; % plot only the x-axis where the problem appears

plotSlackVariables(slack_data.Time, slack_data.pos(i,:), slack_data.vel(i,:), slack_data.accel(i,:), slack_limits);

label_font = 17;
ax_fontsize = 14;

fig = figure;
fig.Position(3:4) = [842 1110];

% ======== position ==========
ax = subplot(3,1,1);
ax_vec = [ax];
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
plot(0, y0(i), 'LineWidth', 4, 'LineStyle','none', 'Color',[0.47,0.67,0.19],'Marker','o', 'MarkerSize',10);
plot(tau, yg(i), 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10);
plot(tau, ygd(i), 'LineWidth', 4, 'LineStyle','none', 'Color','magenta','Marker','x', 'MarkerSize',10);
% plot bounds
plot(ax.XLim, [pos_lim(i,1) pos_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
plot(ax.XLim, [pos_lim(i,2) pos_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
% plot slack bounds
plot(ax.XLim, [pos_lim(i,1) pos_lim(i,1)]-slack_limits(1), 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 0]);
plot(ax.XLim, [pos_lim(i,2) pos_lim(i,2)]+slack_limits(1), 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 0]);
% labels, title ...
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',label_font);
%     title(title_{i}, 'interpreter','latex', 'fontsize',18);
legend(legend_, 'interpreter','latex', 'fontsize',17, 'Position',[0.2330 0.9345 0.5520 0.0294], 'Orientation', 'horizontal');
ax.FontSize = ax_fontsize;
hold off;

% ======== velocity ==========
ax = subplot(3,1,2);
ax_vec = [ax_vec ax];
hold on;
plot(tau, 0, 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10);
for k=1:length(data)
    if (~data{k}.plot2D), continue; end
    plot(data{k}.Time, data{k}.Vel(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color);
end
axis tight;
% plot bounds
plot(ax.XLim, [vel_lim(i,1) vel_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
plot(ax.XLim, [vel_lim(i,2) vel_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
% plot slack bounds
plot(ax.XLim, [vel_lim(i,1) vel_lim(i,1)]-slack_limits(2), 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 0]);
plot(ax.XLim, [vel_lim(i,2) vel_lim(i,2)]+slack_limits(2), 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 0]);
ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',label_font);
ax.FontSize = ax_fontsize;
hold off;

% ======== acceleration ==========
ax = subplot(3,1,3);
ax_vec = [ax_vec ax];
hold on;
plot(tau, 0, 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10);
for k=1:length(data)
    if (~data{k}.plot2D), continue; end
    plot(data{k}.Time, data{k}.Accel(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color);
end
axis tight;
% plot bounds
plot(ax.XLim, [accel_lim(i,1) accel_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
plot(ax.XLim, [accel_lim(i,2) accel_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
% plot slack bounds
plot(ax.XLim, [accel_lim(i,1) accel_lim(i,1)]-slack_limits(3), 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 0]);
plot(ax.XLim, [accel_lim(i,2) accel_lim(i,2)]+slack_limits(3), 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 0]);
%ax.YLim = [ max(ax.YLim(1), 8*accel_lim(i,1)) min(ax.YLim(2), 8*accel_lim(i,2)) ];
ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',label_font);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',label_font);
ax.FontSize = ax_fontsize;
hold off;

linkaxes(ax_vec,'x');



%% ============  Utility functions  ===========

function [Time, P_data, dP_data, ddP_data, slack_data] = gmpMpcOpt(gmp0, Tf, y0, yg, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel, use_slack, slack_limits)

    gmp = gmp0.deepCopy();
   
    n_dof = length(y0);

    O_ndof = zeros(n_dof,1);

    t = 0;
    dt = 0.002;
    tau = Tf;
    y = y0;
    y_dot = O_ndof;
    y_ddot = O_ndof;
    
    can_sys = CanonicalSystem(Tf);
    
    N_horizon = 10;
    pred_time_step = 0.1;
    
    N_kernels = 30;
    kernels_std_scaling = 1.5;
    
    final_state_err_tol = [2e-2; 5e-2; 1e-1];
    
    slack_gains = [0 0 0];
    if (use_slack)
        slack_gains = [1e5 100 1];
        %slack_limits = [2e-2, 0.2, 1.0]; 
    end

    time_limit = 0; %2e-3;
    max_iter = 20000;
    abs_tol = 1e-4;
    rel_tol = 1e-6;
        
    %% --------  GMP - MPC  --------
    gmp_mpc = GMP_MPC(gmp, N_horizon, pred_time_step, N_kernels, kernels_std_scaling, slack_gains);
    
    gmp_mpc.settings.max_iter = max_iter;
    gmp_mpc.settings.time_limit = time_limit;
    gmp_mpc.settings.abs_tol = abs_tol;
    gmp_mpc.settings.rel_tol = rel_tol;
    
    gmp_mpc.setObjCostGains(opt_pos, opt_vel);
    
    gmp_mpc.setPosLimits(pos_lim(:,1), pos_lim(:,2));
    gmp_mpc.setVelLimits(vel_lim(:,1), vel_lim(:,2));
    gmp_mpc.setAccelLimits(accel_lim(:,1), accel_lim(:,2));
    
    if (use_slack)
        gmp_mpc.setPosSlackLimit(slack_limits(1));
        gmp_mpc.setVelSlackLimit(slack_limits(2));
        gmp_mpc.setAccelSlackLimit(slack_limits(3));
    end

    gmp_mpc.setInitialState(y, y_dot, y_ddot, can_sys.s, can_sys.s_dot, 0);
    gmp_mpc.setFinalState(yg, O_ndof, O_ndof, 1, can_sys.s_dot, 0, final_state_err_tol);
    
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
    slack_data.pos = [];
    slack_data.vel = [];
    slack_data.accel = [];
    
    %% set canonical system function
    can_sys_fun = @(s, s_dot) [s_dot; can_sys.getPhaseDDot(s, s_dot)];
    gmp_mpc.setCanonicalSystemFunction( can_sys_fun );

    %% --------  Simulation loop  --------
    while (true)

        if (can_sys.s >= 1), break; end
        
        if (can_sys.s <= 1), text_prog.update(100*can_sys.s); end
        
        if (~opt_fail)
            
            sol = gmp_mpc.solve(can_sys.s, can_sys.s_dot);

            if (sol.exit_flag)
                warning(sol.exit_msg);
                text_prog.printInNewLine();
                if (sol.exit_flag < 0)
                    opt_fail = true;
                    if (on_opt_fail_exit), break; end 
                end
            end
            
        end
        
        y = gmp_mpc.getYd(can_sys.s); %sol.y;
        y_dot = gmp_mpc.getYdDot(can_sys.s, can_sys.s_dot); %sol.y_dot;
        y_ddot = gmp_mpc.getYdDDot(can_sys.s, can_sys.s_dot, can_sys.getPhaseDDot()); %sol.y_ddot;
        % gmp_mpc.setInitialState(y, y_dot, y_ddot, s, s_dot, s_ddot);

        %% --------  Log data  --------
        Time = [Time t];
        P_data = [P_data y];
        dP_data = [dP_data y_dot];
        ddP_data = [ddP_data y_ddot];
        if (use_slack)
           slack_data.pos = [slack_data.pos sol.pos_slack];
           slack_data.vel = [slack_data.vel sol.vel_slack];
           slack_data.accel = [slack_data.accel sol.accel_slack];
        end
        
        %% --------  Numerical integration  --------
        can_sys.integrate(t, t+dt);
        t = t + dt;
    
    end
    
    if (use_slack), slack_data.Time = Time; end

    if (can_sys.s >= 1), text_prog.update(100); end
    
    fprintf('\n');
    fprintf('===> GMP-MPC optimization finished! Elaps time: %f ms\n',toc(t_start)*1000);

end


function [s_dot, s_ddot] = my_can_sys_fun(s, s_dot, sd_dot)
    
    s_dot = s_dot;
    
    if (s < 1), s_ddot = -100*(s_dot - sd_dot);
    else,       s_ddot = -400*s_dot - 800*(s-1);
    end

end

