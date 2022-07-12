clc;
% close all;
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
Tf0 = 7; % initial execution time duration
Tf_new  = 5; % new execution time duration
tf_change = 2; % time instant of change of time duration
vTf = struct('Tf0',Tf0, 'Tf_new',Tf_new, 'tf_change',tf_change, 'a_tf',100);

%% to change online the target pose at t = t_g from 'yg0' to 'yg'
t_g = 3;
yg0 = yg;

y0 = [-0.0956; -0.443; 0.15];

gmp.setY0(y0);
gmp.setGoal(yg0);


%% =============  Run  ==================

data = {};
opt_type = 'p';
if (opt_vel), opt_type = 'v'; end

vtarget = VariableTarget(yg, yg, t_g, 5); % constant target

dt = 0.005;

can_sys = CanonicalSystem(Tf0, 50);

% plot_regressVec_with_variable_phase(vTf);

%% --------- Proportional scaling -----------
gmp.setScaleMethod(TrajScale_Prop(n_dof));
[Time, P_data, dP_data, ddP_data] = getGMPTrajectory(gmp, dt, can_sys, vTf, y0, vtarget);
data{length(data)+1} = ...
    struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle',':', ...
        'color','blue', 'legend',['$' 'DMP' '$'], 'plot3D',true, 'plot2D',true); 

vtarget.reset();
Time_target = Time;    
target_data = zeros(n_dof, length(Time_target));
for j=1:length(Time), target_data(:,j) = vtarget.getTarget(Time(j)); end

target_changed = norm(ygd - yg) ~= 0;

%% ---------- GMP-MPC optimization ------------

vtarget.reset();
[Time, P_data, dP_data, ddP_data] = gmpMpcOpt(gmp, dt, can_sys, vTf, y0, vtarget, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel);

data{length(data)+1} = ...
    struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','-', ...
    'color',[0.72 0.27 1], 'legend','$\overline{DMP}^*$', 'plot3D',true, 'plot2D',true);

%% ======= Plot results =========

Tf = Tf_new;

i = 3; % plot only the z-axis

Time_hat = Time;
ddP_hat_data = [diff(dP_data(i,:))./diff(Time_hat) 0];

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
    plot(data{k}.Time, data{k}.Pos(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color);
    legend_ = [legend_ data{k}.legend];
end
% plot target
if (target_changed)
    plot(Time_target, target_data(i,:), 'LineWidth',2.5, 'LineStyle','-', 'Color',[1 0 0 0.5]);
end
% plot t-instant that duration changes
plot(tf_change*[1 1], ax.YLim, 'LineWidth',1.5, 'LineStyle','--', 'Color',[0 1 0 0.5]);
axis tight;
% plot start and final positions
plot(0, y0(i), 'LineWidth', 4, 'LineStyle','none', 'Color',[0.47,0.67,0.19],'Marker','o', 'MarkerSize',10);
plot(Tf, yg(i), 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10);
plot(Tf, ygd(i), 'LineWidth', 4, 'LineStyle','none', 'Color','magenta','Marker','x', 'MarkerSize',10);
% plot bounds
plot(ax.XLim, [pos_lim(i,1) pos_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
plot(ax.XLim, [pos_lim(i,2) pos_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
% plot hard limits
plot(ax.XLim, [pos_lim(i,1) pos_lim(i,1)]-p_slack, 'LineWidth',2, 'LineStyle','--', 'Color',[0.5 0.5 0.5], 'HandleVisibility','off');
plot(ax.XLim, [pos_lim(i,2) pos_lim(i,2)]+p_slack, 'LineWidth',2, 'LineStyle','--', 'Color',[0.5 0.5 0.5], 'HandleVisibility','off');
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
% plot(Time_hat, ddP_hat_data, 'LineWidth',2, 'LineStyle',':', 'Color',[0 1 0 0.8]);
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

function [Time, P_data, dP_data, ddP_data] = getGMPTrajectory(gmp, dt, can_sys, vTf, y0, vtarget)

    Time = [];
    P_data = [];
    dP_data = [];
    ddP_data = [];

    p = y0;
    p_dot = zeros(size(p));
    p_ddot = zeros(size(p));

    t = 0;
    
    gmp.setY0(y0);
    
    Tf = vTf.Tf0;
    Tf_target = vTf.Tf0;
    
    can_sys.reset();
    can_sys.setDuration(Tf);

    while (true)
        
        if (t >= vTf.tf_change)
            Tf_target = vTf.Tf_new;
            vTf.tf_change = inf;
            if ( vTf.a_tf > 1e3 )
                Tf = Tf_target; 
                vTf.a_tf = 0;
            end
        end
        
        can_sys.setDuration(Tf, t);
        
        Tf_dot = vTf.a_tf * (Tf_target - Tf);
        
        yg = vtarget.getTarget(t);
        gmp.setGoal(yg);

        s_ddot = can_sys.getPhaseDDot(can_sys.s, can_sys.s_dot);

        p_ref = gmp.getYd(can_sys.s);
        p_ref_dot = gmp.getYdDot(can_sys.s, can_sys.s_dot);
        p_ref_ddot = gmp.getYdDDot(can_sys.s, can_sys.s_dot, s_ddot);
        
        p_ddot = 300*(p_ref - p) + 80*(p_ref_dot - p_dot) + p_ref_ddot;
        
        % if the target changes, this will produce erroneous results!
%         p = p_ref;
%         p_dot = p_ref_dot;
%         p_ddot = p_ref_ddot;

        Time = [Time t];
        P_data = [P_data p];
        dP_data = [dP_data p_dot];
        ddP_data = [ddP_data p_ddot];

        can_sys.integrate(t, t+dt);
        t = t + dt;
        p = p + p_dot*dt;
        p_dot = p_dot + p_ddot*dt;
        
        Tf = Tf + Tf_dot*dt;

        if (can_sys.s >= 1.0) % && norm(p_dot)<5e-3 && norm(p_ddot)<1e-2)
            break; 
        end

    end

end

function [Time, P_data, dP_data, ddP_data] = gmpMpcOpt(gmp0, dt, can_sys, vTf, y0, vtarget, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel)
      
    gmp = gmp0.deepCopy();
   
    n_dof = length(y0);

    O_ndof = zeros(n_dof,1);
    
    Tf = vTf.Tf0;
    Tf_target = vTf.Tf0;

    t = 0;
    can_sys.reset();
    can_sys.setDuration(Tf);
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
    
    time_limit = 100e-3;
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
        
        %% Stopping criteria
        if (can_sys.s >= 1), break; end
        
        % display progress
        if (can_sys.s <= 1), text_prog.update(100*can_sys.s); end
        
        if (t >= vTf.tf_change)
            Tf_target = vTf.Tf_new;
            vTf.tf_change = inf;
            if ( vTf.a_tf > 1e3 )
                Tf = Tf_target; 
                vTf.a_tf = 0;
            end
        end
        
        can_sys.setDuration(Tf, t);
        
        Tf_dot = vTf.a_tf * (Tf_target - Tf);
        
        %% update final state
        yg = vtarget.getTarget(t);
        gmp.setGoal(yg);
        gmp_mpc.setFinalState(yg, O_ndof, O_ndof, 1, can_sys.sd_dot, 0, final_state_err_tol);

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
        
%         y = gmp_mpc.getYd(s); %sol.y;
%         y_dot = gmp_mpc.getYdDot(s, s_dot); %sol.y_dot;
%         y_ddot = gmp_mpc.getYdDDot(s, s_dot, s_ddot); %sol.y_ddot; 

        y_ref = gmp_mpc.getYd(s);
        y_ref_dot = gmp_mpc.getYdDot(s, s_dot);
        y_ref_ddot = gmp_mpc.getYdDDot(s, s_dot, s_ddot);
        
%         y_ddot = 300*(y_ref - y) + 80*(y_ref_dot - y_dot) + y_ref_ddot;
        
        y = y_ref;
        y_dot = y_ref_dot;
        y_ddot = y_ref_ddot;
        
        gmp_mpc.setInitialState(y, y_dot, y_ddot, s, s_dot, s_ddot);

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
        Tf = Tf + Tf_dot*dt;
        
        y = y + y_dot*dt;
        y_dot = y_dot + y_ddot*dt;
    
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


function plot_regressVec_with_variable_phase(vTf)

    Time = [];
    s_data = [];
    sdot_data = [];
    sddot_data  = [];

    Tf = vTf.Tf0;
    Tf_target = vTf.Tf0;
    tf_change = vTf.tf_change; % time instant of change of time duration

    t = 0;
    dt = 0.002;
    can_sys = CanonicalSystem(vTf.Tf0, 30);
    gmp = GMP_regressor(30, 1.5);

    while (true)

        if (t > vTf.Tf_new), break; end

        Time = [Time t];
        s_data = [s_data can_sys.s];
        sdot_data = [sdot_data can_sys.s_dot];
        sddot_data  = [sddot_data can_sys.getPhaseDDot(can_sys.s, can_sys.s_dot)];

        if (t >= vTf.tf_change)
            Tf_target = vTf.Tf_new;
            vTf.tf_change = inf;
            if ( vTf.a_tf > 1e3 )
                Tf = Tf_target; 
                vTf.a_tf = 0;
            end
        end

        can_sys.setDuration(Tf, t);

        Tf_dot = vTf.a_tf*(Tf_target - Tf);

        %% Numerical integration
        can_sys.integrate(t, t+dt);
        t = t + dt;
        Tf = Tf + Tf_dot*dt;

    end

    plotRegressVec2(gmp, Time, s_data, sdot_data, sddot_data);
        
end

