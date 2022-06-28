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
    [ -0.7 , 0.3];
    [ 0.05 , 0.6];
  ];

vel_lim = repmat([ -0.4 , 0.4 ], n_dof, 1);

accel_lim = repmat([ -0.8 , 0.8 ], n_dof, 1);

% pos_lim = pos_lim + repmat([-1 1], n_dof, 1); % to augment the limits

%% --------- Optimization objective ----------
opt_pos = 1;
opt_vel = 0;

%% -------- Initial/Final states ---------

ygd = [0.26, -0.12, 0.3]';   % demo target
yg = [ 0.55, 0.0817, 0.46 ]'; % execution target
Tf = 4.5; % execution time duration

%% to change online the target pose at t = t_g from 'yg0' to 'yg'
t_g = 0.5*Tf;
yg0 = ygd;

y0 = [-0.0956; -0.443; 0.15];

gmp.setY0(y0);
gmp.setGoal(yg0);


%% =============  Run  ==================

data = {};
opt_type = 'p';
if (opt_vel), opt_type = 'v'; end

vtarget = VariableTarget(ygd, yg, t_g, 5);

dt = 0.005;

%% --------- Proportional scaling -----------
gmp.setScaleMethod(TrajScale_Prop(n_dof));
[Time, P_data, dP_data, ddP_data] = getGMPTrajectory(gmp, dt, Tf, y0, vtarget);
data{length(data)+1} = ...
    struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle',':', ...
        'color','blue', 'legend',['$' 'DMP' '$'], 'plot3D',true, 'plot2D',true); 

vtarget.reset();
Time_target = Time;    
target_data = zeros(n_dof, length(Time_target));
for j=1:length(Time), target_data(:,j) = vtarget.getTarget(Time(j)); end

    
%% ---------- GMP-MPC optimization ------------

vtarget.reset();
[Time, P_data, dP_data, ddP_data] = gmpMpcOpt(gmp, dt, Tf, y0, vtarget, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel);

data{length(data)+1} = ...
    struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','-', ...
    'color',[0.72 0.27 1], 'legend','$\overline{DMP}^*$', 'plot3D',true, 'plot2D',true);

%% ======= Plot results =========

i = 3; % plot only the z-axis where the problem appears

label_font = 17;
ax_fontsize = 14;

fig = figure;
fig.Position(3:4) = [842 1110];

global slack_limits

p_slack = slack_limits(1);
v_slack = slack_limits(2);
a_slack = slack_limits(3);

% ======== position ==========
ax = subplot(3,1,1);
ax_vec = [ax];
hold on;
% plot position trajectory
legend_ = {};
for k=1:length(data)
    if (~data{k}.plot2D), continue; end
    plot(data{k}.Time, data{k}.Pos(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color, 'DisplayName', data{k}.legend);
    legend_ = [legend_ data{k}.legend];
end
% plot target
plot(Time_target, target_data(i,:), 'LineWidth',2.5, 'LineStyle','-', 'Color',[1 0 0 0.5], 'DisplayName', 'Target update');
axis tight;
% plot start and final positions
plot(0, y0(i), 'LineWidth', 4, 'LineStyle','none', 'Color',[0.47,0.67,0.19],'Marker','o', 'MarkerSize',10, 'HandleVisibility','off');
plot(Tf, yg(i), 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10, 'HandleVisibility','off');
plot(Tf, ygd(i), 'LineWidth', 4, 'LineStyle','none', 'Color','magenta','Marker','x', 'MarkerSize',10, 'HandleVisibility','off');
% plot bounds
plot(ax.XLim, [pos_lim(i,1) pos_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1], 'HandleVisibility','off');
plot(ax.XLim, [pos_lim(i,2) pos_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1], 'HandleVisibility','off');
% plot hard limits
plot(ax.XLim, [pos_lim(i,1) pos_lim(i,1)]-p_slack, 'LineWidth',2, 'LineStyle','--', 'Color',[0.5 0.5 0.5], 'HandleVisibility','off');
plot(ax.XLim, [pos_lim(i,2) pos_lim(i,2)]+p_slack, 'LineWidth',2, 'LineStyle','--', 'Color',[0.5 0.5 0.5], 'HandleVisibility','off');
% labels, title ...
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',label_font);
%     title(title_{i}, 'interpreter','latex', 'fontsize',18);
legend({}, 'interpreter','latex', 'fontsize',17, 'Position',[0.2330 0.9345 0.5520 0.0294], 'Orientation', 'horizontal');
ax.FontSize = ax_fontsize;
hold off;

% ======== velocity ==========
ax = subplot(3,1,2);
ax_vec = [ax_vec ax];
hold on;
plot(Tf, 0, 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10);
for k=1:length(data)
    if (~data{k}.plot2D), continue; end
    plot(data{k}.Time, data{k}.Vel(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color);
end
axis tight;
% plot bounds
plot(ax.XLim, [vel_lim(i,1) vel_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
plot(ax.XLim, [vel_lim(i,2) vel_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
% plot hard limits
plot(ax.XLim, [vel_lim(i,1) vel_lim(i,1)]-v_slack, 'LineWidth',2, 'LineStyle','--', 'Color',[0.5 0.5 0.5], 'HandleVisibility','off');
plot(ax.XLim, [vel_lim(i,2) vel_lim(i,2)]+v_slack, 'LineWidth',2, 'LineStyle','--', 'Color',[0.5 0.5 0.5], 'HandleVisibility','off');
ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',label_font);
ax.FontSize = ax_fontsize;
hold off;

% ======== acceleration ==========
ax = subplot(3,1,3);
ax_vec = [ax_vec ax];
hold on;
plot(Tf, 0, 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10);
for k=1:length(data)
    if (~data{k}.plot2D), continue; end
    plot(data{k}.Time, data{k}.Accel(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color);
end
axis tight;
% plot bounds
plot(ax.XLim, [accel_lim(i,1) accel_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
plot(ax.XLim, [accel_lim(i,2) accel_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
% plot hard limits
plot(ax.XLim, [accel_lim(i,1) accel_lim(i,1)]-a_slack, 'LineWidth',2, 'LineStyle','--', 'Color',[0.5 0.5 0.5], 'HandleVisibility','off');
plot(ax.XLim, [accel_lim(i,2) accel_lim(i,2)]+a_slack, 'LineWidth',2, 'LineStyle','--', 'Color',[0.5 0.5 0.5], 'HandleVisibility','off');
ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',label_font);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',label_font);
ax.FontSize = ax_fontsize;
hold off;

linkaxes(ax_vec,'x');


%% ===========  Utility functions =============

function [Time, P_data, dP_data, ddP_data] = getGMPTrajectory(gmp, dt, tau, y0, vtarget)

    Time = [];
    P_data = [];
    dP_data = [];
    ddP_data = [];

    p = y0;
    p_dot = zeros(size(p));
    p_ddot = zeros(size(p));

    t = 0;
    
    gmp.setY0(y0);
    
    s = 0;

    while (true)
        
        yg = vtarget.getTarget(t);
        gmp.setGoal(yg);

        s_dot = 1/tau;

        p_ref = gmp.getYd(s);
        p_ref_dot = gmp.getYdDot(s, s_dot);
        p_ref_ddot = gmp.getYdDDot(s, s_dot, 0);
        
        p_ddot = 300*(p_ref - p) + 80*(p_ref_dot - p_dot) + p_ref_ddot;
        
        % if the target changes, this will produce erroneous results!
%         p = p_ref;
%         p_dot = p_ref_dot;
%         p_ddot = p_ref_ddot;

        Time = [Time t];
        P_data = [P_data p];
        dP_data = [dP_data p_dot];
        ddP_data = [ddP_data p_ddot];

        t = t + dt;
        s = s + s_dot*dt;
        
        p = p + p_dot*dt;
        p_dot = p_dot + p_ddot*dt;

        if (s >= 1.0) % && norm(p_dot)<5e-3 && norm(p_ddot)<1e-2)
            break; 
        end

    end

end

function [Time, P_data, dP_data, ddP_data] = gmpMpcOpt(gmp0, dt, Tf, y0, vtarget, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel)
      
    gmp = gmp0.deepCopy();
   
    n_dof = length(y0);

    O_ndof = zeros(n_dof,1);

    t = 0;
    tau = Tf;
    can_sys = CanonicalSystem(Tf, 30);
    y = y0;
    y_dot = O_ndof;
    y_ddot = O_ndof;
    
    yg = vtarget.getTarget(t);
    
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
        yg = vtarget.getTarget(t);
        gmp.setGoal(yg);
        gmp_mpc.setFinalState(yg, O_ndof, O_ndof, 1, can_sys.sd_dot, 0, final_state_err_tol);
            
        %% Stopping criteria
%         if (s >= 1 && norm(y_dot)<1e-2 && norm(y-yg)<1e-2 )
%             break;
%         elseif (s >= 1.3)
%             warning('Time limits exceeded...');
%             break;
%         end
        if (can_sys.s >= 1), break; end
        
        % display progress
        if (can_sys.s <= 1), text_prog.update(100*can_sys.s); end
        
        
        
        %% optimization
        if (~opt_fail)
            
            % sovle
            sol = gmp_mpc.solve(can_sys.s, can_sys.s_dot);

            % check exit status
            if (sol.exit_flag)
                warning(sol.exit_msg);
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


