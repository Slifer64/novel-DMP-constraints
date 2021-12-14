function [Time, P_data, dP_data, ddP_data] = offlineGMPweightsOpt(gmp0, tau, y0, yg, pos_lim, vel_lim, accel_lim, opt_pos, opt_vel, vp_config, qp_solver_type)
    
    gmp = gmp0.deepCopy();
    
    gmp.autoRetrain(30, 1.5, 300, 'LS');
    
    n_dof = length(y0);
    
    gmp.setScaleMethod(TrajScale_Prop(n_dof));
%     gmp.setScaleMethod(TrajScale_Rot_min());
    
    gmp.setY0(y0);
    gmp.setGoal(yg);

    gmp_opt = GMP_Opt(gmp);
    gmp_opt.setOptions(opt_pos, opt_vel, 0);
    gmp_opt.setMotionDuration(tau);
    
    pos_constr.s = [0 1];
    pos_constr.value = [y0 yg];
    
    if (~isempty(vp_config))
        vp = vp_config.via_points;
        for i=1:length(vp)
            pos_constr.s = [pos_constr.s vp{i}.s];
            pos_constr.value = [pos_constr.value vp{i}.pos];
        end
    end
    
    n_points = 200;
    % position constr
    gmp_opt.setPosBounds(pos_lim(:,1), pos_lim(:,2), n_points);
    gmp_opt.setPosConstr([],[],[], pos_constr.s, pos_constr.value);
    % velocity constr
    gmp_opt.setVelBounds(vel_lim(:,1), vel_lim(:,2), n_points);
    gmp_opt.setVelConstr([], [], [], [0 1], zeros(n_dof,2));
    % accel constr
    gmp_opt.setAccelBounds(accel_lim(:,1), accel_lim(:,2), n_points);
    gmp_opt.setAccelConstr([], [], [], [0 1], zeros(n_dof,2));
    
    n_points

    % gmp_opt.optimize(100);
    t_start = tic;
    if (qp_solver_type == 0), gmp_opt.setQPsolver(GMP_Opt.MATLAB_QUADPROG);
    elseif (qp_solver_type == 1), gmp_opt.setQPsolver(GMP_Opt.OSQP);
    elseif (qp_solver_type == 2), gmp_opt.setQPsolver(GMP_Opt.GOLDFARB_IDNANI);
    end
    gmp_opt.optimize2( linspace(0,1, 200) );
    elaps_t = toc(t_start);
    fprintf('===> offline-GMP-weights optimization finished! Elaps time: %f ms\n',elaps_t*1000);
    fprintf([gmp_opt.getExitMsg() '\n']);

    [Time, P_data, dP_data, ddP_data] = getGMPTrajectory(gmp, tau, y0, yg);
    
end
