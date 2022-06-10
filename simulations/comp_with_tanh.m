clc;
close all;
clear;

addpath('utils/');
import_gmp_lib();
import_io_lib();

%% Load training data
fid = FileIO('data/pos_data.bin', FileIO.in);
Timed = fid.read('Timed');
Pd_data = fid.read('Pd_data');
dPd_data = fid.read('dPd_data');
ddPd_data = fid.read('ddPd_data');
fid.close();

ind = [3];

kt = 0.5;
Timed = Timed * kt;
sd_data = Timed / Timed(end);
yd_data = Pd_data(ind, :);
dyd_data = dPd_data(ind, :) / kt;
ddyd_data = ddPd_data(ind, :) / kt^2;

ygd = yd_data(:, end);

n_dofs = size(yd_data, 1);

pos_lim = [0.12 0.7];
vel_lim = [-1.0 1.0];
accel_lim = [-4.5 4.5];


%% GMP
gmp = GMP(n_dofs, 30, 1.5);
train_err = gmp.train('LS', sd_data, yd_data)


%% ksi GMP
ksi_data = ksi_transform(yd_data, pos_lim);
gmp_ksi = GMP(n_dofs, 30, 1.5);
ksi_train_err = gmp_ksi.train('LS', sd_data, ksi_data)


%% Simulation
y0 = yd_data(:, 1);
yg = ygd + 0.23;
Tf = Timed(end);
s_dot = 1 / Tf;
s_ddot = 0;

gmp.setGoal(yg);
gmp.setY0(y0);

gmp_ksi.setGoal(ksi_transform(yg, pos_lim));
gmp_ksi.setY0(ksi_transform(y0, pos_lim));

y_data = zeros(n_dofs, length(sd_data));
dy_data = zeros(n_dofs, length(sd_data));
ddy_data = zeros(n_dofs, length(sd_data));

y2_data = zeros(n_dofs, length(sd_data));
dy2_data = zeros(n_dofs, length(sd_data));
ddy2_data = zeros(n_dofs, length(sd_data));
for j=1:length(sd_data)
    s = sd_data(j);
    
    y_data(:,j) = gmp.getYd(s);
    dy_data(:,j) = gmp.getYdDot(s, s_dot);
    ddy_data(:,j) = gmp.getYdDDot(s, s_dot, s_ddot);
    
    ksi = gmp_ksi.getYd(s);
    ksi_dot = gmp_ksi.getYdDot(s, s_dot);
    y2_data(:,j) = inv_ksi_transform(ksi, pos_lim);
    dy2_data(:,j) = ksi_Jacob(ksi, pos_lim) * ksi_dot;
end
ddy2_data = [diff(dy2_data)./diff(Timed) 0]; 


[Time3, y3_data, dy3_data, ddy3_data] = gmpMpcOpt(gmp, Timed(end), y0, yg, pos_lim, vel_lim, accel_lim, 1, 0);

% [Time4, y4_data, dy4_data, ddy4_data] = gmpMpcOpt(gmp, Timed(end), y0, yg, pos_lim, vel_lim, accel_lim, 0, 1);

%% Plot results

fig = figure;
fig.Position(3:4) = [667 878];
% ----------------------------
ax=subplot(3,1,1); hold(ax, 'on');
plot(Timed, yd_data, 'Color','cyan', 'LineWidth',2, 'DisplayName','demo');
plot(Timed, y_data, 'Color','blue', 'LineStyle','-', 'LineWidth',2, 'DisplayName','DMP');
plot(Timed, y2_data, 'Color','magenta', 'LineStyle','-', 'LineWidth',2, 'DisplayName','DMP-tanh');
plot(Time3, y3_data, 'Color','green', 'LineStyle','-.', 'LineWidth',2, 'DisplayName','$\overline{DMP}_p^*$');
% plot(Time4, y4_data, 'Color',[0 0.5 0], 'LineStyle',':', 'LineWidth',2, 'DisplayName','$\overline{DMP}_v^*$');
plot(Timed(1), y0, 'Marker','o', 'MarkerSize',10, 'LineWidth',3, 'color',[0 0.6 0], 'LineStyle','None', 'HandleVisibility','off');
plot(Timed(end), yg, 'Marker','x', 'MarkerSize',10, 'LineWidth',3, 'color',[0.8 0 0], 'LineStyle','None', 'HandleVisibility','off');
plot(Timed(end), ygd, 'Marker','x', 'MarkerSize',10, 'LineWidth',3, 'color',[1 0.6 0.6], 'LineStyle','None', 'HandleVisibility','off');
plot([Timed(1) Timed(end)], [pos_lim(1) pos_lim(1)], 'color',[1, 0.5, 0.5], 'LineStyle','--', 'LineWidth',2, 'DisplayName','limits');
plot([Timed(1) Timed(end)], [pos_lim(end) pos_lim(end)], 'color',[1, 0.5, 0.5], 'LineStyle','--', 'LineWidth',2, 'HandleVisibility','off');
legend('interpreter','latex', 'fontsize',15, 'Position',[0.1194 0.9401 0.7925 0.0354], 'Orientation','horizontal');
ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',16);
axis(ax, 'tight');
% ----------------------------
ax=subplot(3,1,2); hold(ax, 'on');
plot(Timed, dy2_data, 'Color','magenta', 'LineStyle','-', 'LineWidth',2);
plot(Time3, dy3_data, 'Color','green', 'LineStyle','-.', 'LineWidth',2);
plot(Timed(end), 0, 'Marker','x', 'MarkerSize',10, 'LineWidth',3, 'color',[0.8 0 0], 'LineStyle','None');
plot([Timed(1) Timed(end)], [vel_lim(1) vel_lim(1)], 'color',[1, 0.5, 0.5], 'LineStyle','--', 'LineWidth',2, 'DisplayName','limits');
plot([Timed(1) Timed(end)], [vel_lim(end) vel_lim(end)], 'color',[1, 0.5, 0.5], 'LineStyle','--', 'LineWidth',2, 'HandleVisibility','off');
ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',16);
axis(ax, 'tight');
% ----------------------------
ax=subplot(3,1,3); hold(ax, 'on');
plot(Timed, ddy2_data, 'Color','magenta', 'LineStyle','-', 'LineWidth',2);
plot(Time3, ddy3_data, 'Color','green', 'LineStyle','-.', 'LineWidth',2);
plot(Timed(end), 0, 'Marker','x', 'MarkerSize',10, 'LineWidth',3, 'color',[0.8 0 0], 'LineStyle','None');
plot([Timed(1) Timed(end)], [accel_lim(1) accel_lim(1)], 'color',[1, 0.5, 0.5], 'LineStyle','--', 'LineWidth',2, 'DisplayName','limits');
plot([Timed(1) Timed(end)], [accel_lim(end) accel_lim(end)], 'color',[1, 0.5, 0.5], 'LineStyle','--', 'LineWidth',2, 'HandleVisibility','off');
ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',16);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',16);
axis(ax, 'tight');

%% ===========================================
%% ===========================================

function ksi = ksi_transform(y, y_lim)

    y_min = y_lim(1);
    y_max = y_lim(2);
    
    y_delta = 0.5* diag(y_max - y_min);
    yo = 0.5* (y_max + y_min);

    ksi = atanh(inv(y_delta)*(y-yo));

end

function y = inv_ksi_transform(ksi, y_lim)

    y_min = y_lim(1);
    y_max = y_lim(2);
    
    y_delta = 0.5 * diag(y_max - y_min);
    yo = 0.5 * (y_max + y_min);

    y = y_delta * tanh(ksi) + yo;

end

function J = ksi_Jacob(ksi, y_lim)

    y_min = y_lim(1);
    y_max = y_lim(2);
    
    y_delta = 0.5 * diag(y_max - y_min);
    yo = 0.5 * (y_max + y_min);

    J = y_delta*(1 -tanh(ksi).^2);

end


function [Time, P_data, dP_data, ddP_data] = gmpMpcOpt(gmp0, Tf, y0, yg, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel)
      
    
    gmp = gmp0.deepCopy();
   
    n_dof = length(y0);

    O_ndof = zeros(n_dof,1);

    t = 0;
    dt = 0.005;
    tau = Tf;
    can_sys = CanonicalSystem(tau, 30);
    y = y0;
    y_dot = O_ndof;
    y_ddot = O_ndof;
    
    N_horizon = 10;
    pred_time_step = 0.1;
    N_kernels = 30;
    kernels_std_scaling = 1.5;
    
    final_state_err_tol = [1e-4; 1e-3; 1e-1];
    
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

    gmp_mpc.setInitialState(y, y_dot, y_ddot, can_sys.s, can_sys.s_dot, 0);
    gmp_mpc.setFinalState(yg, O_ndof, O_ndof, 1, can_sys.s_dot, 0, final_state_err_tol);
    
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
        
        y_ref = gmp_mpc.getYd(s); %sol.y;
        y_ref_dot = gmp_mpc.getYdDot(s, s_dot); %sol.y_dot;
        y_ref_ddot = gmp_mpc.getYdDDot(s, s_dot, s_ddot); %sol.y_ddot;
        
%         y_ddot = 300*(y_ref - y) + 80*(y_ref_dot - y_dot) + y_ref_ddot;
        y = y_ref;
        y_dot = y_ref_dot;
        y_ddot = y_ref_ddot;
        
        %% update initial state (optionally, if say a perturbation occurs)
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
        
        y = y + y_dot*dt;
        y_dot = y_dot + y_ddot*dt;
    
    end

    if (can_sys.s >= 1), text_prog.update(100); end

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



