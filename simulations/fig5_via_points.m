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
    [ -0.2 , 0.65]; % [ -0.65 , 0.65];
    [ -0.7 , 0.3];
    [ 0.05 , 0.7];
  ];

vel_lim = repmat([ -0.5 , 0.5 ], n_dof, 1);

accel_lim = repmat([ -1.5 , 1.5 ], n_dof, 1);

% pos_lim = pos_lim + repmat([-1 1], n_dof, 1); % to augment the limits

%% --------- Optimization objective ----------
opt_pos = 0;
opt_vel = 1;

%% -------- Initial/Final states ---------

ygd = [0.26, -0.12, 0.32]';   % demo target
yg = [ 0.55, 0.0817, 0.46 ]'; % execution target
tau = 7; % execution time duration

%% to change online the target pose at t = t_g from 'yg0' to 'yg'
t_g = inf; %0.5*tau;
yg0 = yg;

y0 = [-0.0956; -0.443; 0.196];

gmp.setY0(y0);
gmp.setGoal(yg0);

%% -------- via-points ----------
t_vp_ahead = tau/4; % introduce each via point 1000 ms before its timestamp
err_tol = 1e-3;
vp = {};
vp{1} = struct('s',0.2, 'pos', [0.1; 0.2; 0.1] , 'err_tol',err_tol);
vp{2} = struct('s',0.4, 'pos', [0.1; -0.3; 0.1], 'err_tol',err_tol);
vp{3} = struct('s',0.3, 'pos', [0.15; 0.01; 0.02] , 'err_tol',err_tol);

gmp.setY0(y0);
gmp.setGoal(yg0);
for i=1:length(vp), vp{i}.pos = gmp.getYd(vp{i}.s) + vp{i}.pos; end

vp_config.t_ahead = t_vp_ahead;
vp_config.via_points = vp;

%% ------- QP solver ---------
qp_solver_type = 0; % matlab-quadprog:0 , OSQP:1, Goldfarb-Idnani: 2

%% ------ Methods to use -----------
demo     = 1; % plot demo
dmp_prop = 1; % original DMP
dmp_opt  = 1; % optimal GMP
dmp_mpc  = 1; % MPC GMP

%% ==========  Run simulations  =============

data = {};

opt_type = 'p';
if (opt_vel), opt_type = 'v'; end

%% --------- Demo -----------
if (demo)
    gmp.setScaleMethod(TrajScale_Prop(n_dof));
    [Time, P_data, dP_data, ddP_data] = getGMPTrajectory(gmp, tau, y0, ygd);
    data{length(data)+1} = ...
        struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle',':', ...
            'color',[0.6 0.6 0.6], 'legend',['$' 'demo' '$'], 'plot3D',true, 'plot2D',false); 
end

%% --------- Proportional scaling -----------
if (dmp_prop)
    gmp.setScaleMethod(TrajScale_Prop(n_dof));
    [Time, P_data, dP_data, ddP_data] = getGMPTrajectory(gmp, tau, y0, yg0, yg, t_g);
    data{length(data)+1} = ...
        struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle',':', ...
            'color','blue', 'legend',['$' 'DMP' '$'], 'plot3D',true, 'plot2D',true); 
end

%% --------- Offline GMP-weights optimization -----------
if (dmp_opt)
    [Time, P_data, dP_data, ddP_data] = offlineGMPweightsOpt(gmp, tau, y0, yg, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel, vp_config, qp_solver_type);
    data{length(data)+1} = ...
        struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle',':', ...
            'color',[0.64,0.08,0.18], 'legend',['$' 'DMP^*_' opt_type '$'], 'plot3D',true, 'plot2D',true);
end


%% ---------- GMP-MPC optimization ------------
if (dmp_mpc)
    [Time, P_data, dP_data, ddP_data] = gmpMpcOpt(gmp, tau, y0, yg0, yg, t_g, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel, vp_config);
    data{length(data)+1} = ...
        struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','-', ...
        'color',[0.72 0.27 1], 'legend',['$' '\overline{DMP}^*_' opt_type '$'], 'plot3D',true, 'plot2D',true);
    
%     [Time, P_data, dP_data, ddP_data] = gmpMpcOpt(gmp, tau, y0, yg0, yg, t_g, pos_lim, 100*vel_lim, 100*accel_lim, opt_pos, opt_vel, vp_config);
%     data{length(data)+1} = ...
%         struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','-', ...
%         'color',[0.85 0.33 0.1], 'legend',['$' '\overline{DMP}^*_' opt_type ' unlimited$'], 'plot3D',true, 'plot2D',true);
end


%% ========  Plot results ===========

view_ = [19.75, 8.62];

plot_results(data, y0, yg, ygd, tau, pos_lim, vel_lim, accel_lim, vp_config.via_points, view_, false);

