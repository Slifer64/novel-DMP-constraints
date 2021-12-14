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
opt_vel = (1 - opt_pos);

%% -------- Initial/Final states ---------

ygd = [0.26, -0.12, 0.32]';   % demo target
yg = [ 0.55, 0.0817, 0.46 ]'; % execution target
tau = 3.5; % execution time duration

%% to change online the target pose at t = t_g from 'yg0' to 'yg'
t_g = inf; %0.5*tau;
yg0 = yg;

y0 = [-0.0956; -0.443; 0.196];

gmp.setY0(y0);
gmp.setGoal(yg0);

%% -------- via-points ----------
vp_config.t_ahead = [];
vp_config.via_points = {};

%% ------- QP solver ---------
qp_solver_type = 0; % matlab-quadprog:0 , OSQP:1, Goldfarb-Idnani: 2

%% ------ Methods to use -----------
demo     = 1; % plot demo
dmp_prop = 1; % original DMP
dmp_mpc  = 1; % MPC GMP
dmp_rf   = 1; % original DMP with rep-forces

%% =============  Run  ==================

opt_type = 'p';
if (opt_vel), opt_type = 'v'; end

data = {};

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


%% ---------- GMP-MPC optimization ------------
if (dmp_mpc)
    [Time, P_data, dP_data, ddP_data] = gmpMpcOpt(gmp, tau, y0, yg0, yg, t_g, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel, vp_config);
    data{length(data)+1} = ...
        struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','-', ...
        'color',[0.72 0.27 1], 'legend',['$' '\overline{DMP}^*_' opt_type '$'], 'plot3D',true, 'plot2D',true);
end

%% ---------- GMP with repulsive forces ------------
if (dmp_rf)
    [Time, P_data, dP_data, ddP_data] = gmpWithRepulsiveForces(gmp, tau, y0, yg0, yg, t_g, pos_lim, vel_lim, accel_lim);
    data{length(data)+1} = ...
        struct('Time',Time, 'Pos',P_data, 'Vel',dP_data, 'Accel',ddP_data, 'linestyle','-', ...
        'color',[0.93 0.69 0.13], 'legend',['$' 'DMP-RF' '$'], 'plot3D',true, 'plot2D',true);
end


%% ======= Plot results =========

plot_results(data, y0, yg, ygd, tau, pos_lim, vel_lim, accel_lim, [], [], true);
