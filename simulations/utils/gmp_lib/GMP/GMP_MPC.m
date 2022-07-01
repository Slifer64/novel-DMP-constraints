classdef GMP_MPC < handle
   
    %% ===============================
    %% ========== Public =============
    %% ===============================
    
    methods (Access = public)
       
        function this = GMP_MPC(gmp, N_horizon, pred_time_step, N_kernels, kernel_std_scaling, slack_gains, trunc_kern_thres)

            if (nargin < 7), trunc_kern_thres = 1e-6; end
            
            this.can_sys_fun = [];
            
            this.obstacles = {};
            
            this.settings = struct('max_iter',8000, 'time_limit',0, 'abs_tol',1e-3, 'rel_tol',1e-4);
            
            this.n_dof = gmp.numOfDoFs();
            
            this.I_ndof = eye(this.n_dof);
            
            this.gmp_ref = gmp;
            
            this.gmp_mpc = GMP(this.n_dof, N_kernels, kernel_std_scaling);
            this.gmp_mpc.setTruncatedKernels(trunc_kern_thres);
                        
            %this.gmp_mpc.plotRegressVec(-0.1:0.01:1.5);
            %stop
            
            this.N = N_horizon;
            this.N_obst = this.N;
            this.dt_ = pred_time_step*ones(1,this.N);

            this.pos_slack = slack_gains(1) > 0;
            this.vel_slack = slack_gains(2) > 0;
            this.accel_slack = slack_gains(3) > 0;
            this.n_slack = this.n_dof*(this.pos_slack + this.vel_slack + this.accel_slack);
            
            this.n_var = this.n_dof * N_kernels + this.n_slack;
            
            this.Aineq_slack = [];
            this.Q_slack = [];
            if (this.pos_slack)
                this.Q_slack = blkdiag(this.Q_slack, slack_gains(1)*this.I_ndof); 
                this.Aineq_slack = [this.Aineq_slack [-this.I_ndof; zeros(2*this.n_dof,this.n_dof)] ];
            end
            if (this.vel_slack)
                this.Q_slack = blkdiag(this.Q_slack, slack_gains(2)*this.I_ndof);
                this.Aineq_slack = [this.Aineq_slack [zeros(this.n_dof); -this.I_ndof; zeros(this.n_dof)] ];
            end
            if (this.accel_slack)
                this.Q_slack = blkdiag(this.Q_slack, slack_gains(3)*this.I_ndof);
                this.Aineq_slack = [this.Aineq_slack [zeros(2*this.n_dof,this.n_dof); -this.I_ndof] ];
            end
            this.Q_slack = sparse(this.Q_slack);
            this.Aineq_slack = sparse(this.Aineq_slack);
    
            this.setObjCostGains(1.0, 0.0);

            n_dof3 = 3*this.n_dof;
            this.Z0 = zeros(this.n_dof*N_kernels + this.n_slack,1);
            this.Z0_dual_ineq = zeros(this.N*n_dof3 + this.n_slack, 1);
            this.Z0_dual_eq = zeros(n_dof3, 1);
            
            this.setPosLimits(-inf(this.n_dof,1), inf(this.n_dof,1));
            this.setVelLimits(-inf(this.n_dof,1), inf(this.n_dof,1));
            this.setAccelLimits(-inf(this.n_dof,1), inf(this.n_dof,1));
            
            this.setPosSlackLimit(inf);
            this.setVelSlackLimit(inf);
            this.setAccelSlackLimit(inf);
            
            this.W_mpc = zeros(this.n_dof, N_kernels);
            
            this.setObjShiftThres(-1e30);
            
%             x_data = linspace(0,1, 300);
%             Yd_data = zeros(this.n_dof, length(x_data));
%             for j=1:length(x_data), Yd_data(:,j) = gmp.getYd(x_data(j)); end
%             this.gmp_mpc.train('LS', x_data, Yd_data);
%             this.W_mpc = this.gmp_mpc.W;
%             W_temp = this.W_mpc';
%             this.Z0(1:this.n_dof*N_kernels) = W_temp(:);

%             this.A_sp = 0;
%             this.H_sp = 0;

        end
        
        function setObjCostGains(this, pos_gain, vel_gain)

            if (pos_gain < 0), error('"pos_gain" must be non-negative.'); end
            if (vel_gain < 0), error('"vel_gain" must be non-negative.'); end
            
            this.pos_gain = pos_gain;
            this.vel_gain = vel_gain;
            this.Qi = blkdiag( pos_gain*this.I_ndof, vel_gain*this.I_ndof );
            this.QN = this.Qi; % blkdiag( 100*I_ndof, 1*I_ndof );
            
        end
        
        function setObjShiftThres(this, dist_thres)
            
            this.obj_shift_thres = dist_thres;
            
        end
        
        function setInitialState(this, y0, y0_dot, y0_ddot, s, s_dot, s_ddot)

            phi0 = this.gmp_mpc.regressVec(s);
            phi0_dot = this.gmp_mpc.regressVecDot(s, s_dot);
            phi0_ddot = this.gmp_mpc.regressVecDDot(s, s_dot, s_ddot);
            this.Phi0 = sparse([kron(this.I_ndof,phi0'); kron(this.I_ndof,phi0_dot'); kron(this.I_ndof,phi0_ddot')]);
            this.x0 = [y0; y0_dot; y0_ddot];
            
        end
        
        function setFinalState(this, yf, yf_dot, yf_ddot, s, s_dot, s_ddot, err_tol)
            
            if (nargin < 8), err_tol = zeros(3,1); end

            this.s_f = s;
            this.err_tol_f = [err_tol(1)*ones(this.n_dof,1); err_tol(2)*ones(this.n_dof,1); err_tol(3)*ones(this.n_dof,1)];
            phi_f = this.gmp_mpc.regressVec(s);
            phi_f_dot = this.gmp_mpc.regressVecDot(s, s_dot);
            phi_f_ddot = this.gmp_mpc.regressVecDDot(s, s_dot, s_ddot);
            this.Phi_f = sparse([kron(this.I_ndof,phi_f'); kron(this.I_ndof,phi_f_dot'); kron(this.I_ndof,phi_f_ddot')]);
            this.x_f = [yf; yf_dot; yf_ddot];
            
        end
        
        function addEllipsoidObstacle(this, c, Sigma)
            
            this.obstacles = [this.obstacles {struct('c',c, 'inv_Sigma',inv(Sigma))}];
            
        end
        
        function addViaPoint(this, s, pos, err_tol)
            
            if (nargin < 4), err_tol = 1e-6; end
            
            vp = struct('s',s, 'pos',pos, 'err_tol',err_tol);

            % find where to insert the new via-point, so that they are sorted
            n = length(this.via_points);
            for i=n:-1:1 % search backwards, since new via-points will probably refer to future time
               if (s > this.via_points{i}.s), break; end
            end
            
            if (i==n), this.via_points{n+1} = vp;
            else, this.via_points = [this.via_points(1:i) {vp} this.via_points(i+1:end)];
            end

        end
        
        function setPosLimits(this, lb, ub)
            
            this.pos_lb = lb;
            this.pos_ub = ub;
            
        end
        
        function setVelLimits(this, lb, ub)
            
            this.vel_lb = lb;
            this.vel_ub = ub;
            
        end
        
        function setAccelLimits(this, lb, ub)
            
            this.accel_lb = lb;
            this.accel_ub = ub;
            
        end
        
        function setPosSlackLimit(this, s_lim)
            
            this.pos_slack_lim = s_lim*ones(this.n_dof,1);
        end
        
        function setVelSlackLimit(this, s_lim)
            
            this.vel_slack_lim = s_lim*ones(this.n_dof,1);
        end
        
        function setAccelSlackLimit(this, s_lim)
            
            this.accel_slack_lim = s_lim*ones(this.n_dof,1);
        end

        function setCanonicalSystemFunction(this, can_sys_fun)
           
            this.can_sys_fun = can_sys_fun;
            
        end
        
        function [si_data, si_dot_data] = integratePhase(this, s0, s0_dot)
            ti = zeros(1,this.N);
            for i=2:length(ti), ti(i) = ti(i-1) + this.dt_(i-1); end
            can_sys_ode_fun = @(t,state) this.can_sys_fun(state(1), state(2));
            [~,state_data] = ode15s(can_sys_ode_fun, ti, [s0; s0_dot]);
            si_data = state_data(:,1)';
            si_dot_data = state_data(:,2)';
        end
        
        function ret = process_via_points(this, s, s_dot)
            
            %% =======  process via-points  =======
            vp_ind = [];
            % find active via-point indices
            for i=1:length(this.via_points)
                if (this.via_points{i}.s >= s), vp_ind = [vp_ind i]; end
            end
            
            % discard past (non active) via-points
            this.via_points = this.via_points(vp_ind);

            % consider only up to the first 3 active via-points 
            n_vp = min(3, length(this.via_points) );
            
            n_dof3 = 3 * this.n_dof;
            
            Aineq = sparse(n_dof3, this.n_var);
            lb = zeros(n_dof3, 1);
            ub = zeros(n_dof3, 1);
            H = sparse(this.n_var, this.n_var);
            q = zeros(this.n_var,1);
            
            for i=1:n_vp
                
                i1 = (i-1)*n_dof3 + 1;
                i2 = i*n_dof3;
                
                vp = this.via_points{i};
                phi = this.gmp_mpc.regressVec(vp.s);
                phi_dot = this.gmp_mpc.regressVecDot(vp.s, s_dot);
                phi_ddot = this.gmp_mpc.regressVecDDot(vp.s, s_dot, 0); % ? s_ddot
                
%                 A_vp = [kron(speye(this.n_dof),phi'), sparse(this.n_dof, this.n_slack)];
%                 lb_vp = vp.pos - vp.err_tol;
%                 ub_vp = vp.pos + vp.err_tol;

                Aineq(i1:i2, :) = [sparse([kron(this.I_ndof,phi'); kron(this.I_ndof,phi_dot'); kron(this.I_ndof,phi_ddot')]), ...
                                   sparse(n_dof3, this.n_slack)];
                lb(i1:i2) = [vp.pos - vp.err_tol; this.vel_lb; this.accel_lb];
                ub(i1:i2) = [vp.pos + vp.err_tol; this.vel_ub; this.accel_ub];

                % add via-point to cost function too, to help the optimizer
                Psi = sparse(kron(this.I_ndof,phi'));
                xd_i = vp.pos;
                Qi_ = 1e3;
                H = H + sparse( blkdiag(Psi'*Qi_*Psi, zeros(this.n_slack)) );
                q = q - [Psi'*Qi_*xd_i; zeros(this.n_slack,1)];
                
%                 A_vp = [sparse([kron(this.I_ndof,phi_dot'); kron(this.I_ndof,phi_ddot')]), ...
%                         sparse(2*this.n_dof, this.n_slack)];
%                 lb_vp = [this.vel_lb; this.accel_lb];
%                 ub_vp = [this.vel_ub; this.accel_ub];

                
            end
            
            ret = struct('Aineq',Aineq, 'lb',lb, 'ub',ub, 'H',H, 'q',q);
            
        end
        
        function [Ai, bi] = process_obstacles(this, p1, phi)
            
            %% check collisions with ellipsoids and calculate plane constraints to avoid them
            Ai = zeros(1, this.n_var);
            bi = -1;
            for k=1:length(this.obstacles)
                c = this.obstacles{k}.c;
                inv_Sigma = 0.9*this.obstacles{k}.inv_Sigma;
                temp = (p1-c)'*inv_Sigma*(p1-c);
                if temp < 1.1
                    %% Find the point on the ellipsoid surface
                    p2 = c + (p1-c) / sqrt(temp);
                    %% Find the norm to the ellipsoid on that point
                    n_e = inv_Sigma*(p2 - c);
                    n_e = n_e / norm(n_e);
                    a_e = phi*n_e';
                    Ai = 0.1*[a_e(:)', zeros(1, this.n_slack)];
                    bi = 0.1*dot(n_e, p2);
                    
                    this.log_.n_e_data = [this.log_.n_e_data {n_e}];
                    this.log_.p_e_data = [this.log_.p_e_data {p2}];
                    this.log_.p_data = [this.log_.p_data {p1}];
                    this.log_.c_data = [this.log_.c_data {c}];

                    break;
                end
            end
            
        end

        function solution = solve(this, s, s_dot)
                
            if (isempty(this.can_sys_fun))
                error('The canonical system function has to be set at least once!');
            end
                
            solution = struct('y',[], 'y_dot',[], 'y_ddot',[], ...
                            'pos_slack', [], 'vel_slack',[], 'accel_slack',[], ...
                            'exit_msg','', 'exit_flag',-1);
            
            N_kernels = this.gmp_mpc.numOfKernels();
            n_dof3 = 3*this.n_dof; % for pos, vel, accel
            
            %% =======  initialize  =======
            H = 1e-6*speye(this.n_var); % for numerical stability
            q = zeros(this.n_var,1);
            N_bounds = this.N*n_dof3;
            Aineq = sparse(N_bounds + this.N_obst + this.n_slack, this.n_var);
            Aineq(end-this.n_slack+1:end,end-this.n_slack+1:end) = speye(this.n_slack);
            
            lb_i = [this.pos_lb; this.vel_lb; this.accel_lb];
            ub_i = [this.pos_ub; this.vel_ub; this.accel_ub];
            
            slack_lim = [];
            if (this.pos_slack), slack_lim = [slack_lim; this.pos_slack_lim];  end
            if (this.vel_slack), slack_lim = [slack_lim; this.vel_slack_lim];  end
            if (this.accel_slack), slack_lim = [slack_lim; this.accel_slack_lim];  end
            
            lb = [repmat(lb_i, this.N,1); -1e30*ones(this.N_obst, 1); -slack_lim];
            ub = [repmat(ub_i, this.N,1); 1e30*ones(this.N_obst, 1); slack_lim];

            % DMP phase variable
            [si_data, si_dot_data] = this.integratePhase(s, s_dot);
            
            A_obst = zeros(this.N_obst, this.n_var);
            b_obst = zeros(this.N_obst, 1);

            this.log_.clear();

            %% =======  calc cost and inequality constraints along the horizon N  =======
            for i=1:this.N
                
                si = si_data(i);
                si_dot = si_dot_data(i);
                s_temp = this.can_sys_fun(si, si_dot);
                si_ddot = s_temp(2);

                yd_i = this.gmp_ref.getYd(si);
                dyd_i = this.gmp_ref.getYdDot(si, si_dot);

                phi = this.gmp_mpc.regressVec(si);
                phi_dot = this.gmp_mpc.regressVecDot(si, si_dot);
                phi_ddot = this.gmp_mpc.regressVecDDot(si, si_dot, si_ddot);
                
                yg = this.x_f(1:this.n_dof);
                target_dist = min([norm(this.getYd(si) - yg), norm(yd_i - yg)]);
                if (target_dist < this.obj_shift_thres)
                    %lambda = 1 - exp(-0.5*this.obj_shift_gain/target_dist);
                    lambda = 1 - (target_dist/this.obj_shift_thres);
                    this.Qi = blkdiag(lambda*this.I_ndof, (1-lambda)*this.I_ndof);
                end
                   
                if (i==this.N), Qi_ = this.QN;
                else, Qi_ = this.Qi;
                end
                
                
                %% check collisions with ellipsoids and calculate plane constraints to avoid them
                y = this.getYd(si);
                [Ai, bi] = this.process_obstacles(y, phi);
                %[Ai, bi] = this.process_obstacles(yd_i, phi);
                A_obst(i,:) = Ai;
                b_obst(i) = bi;
                
                this.log_.yd_points = [this.log_.yd_points yd_i];
                this.log_.y_pred_points = [this.log_.y_pred_points y];

                % since the gmp model is valid in [0 1]. Beyond that it
                % might produce small errors
                if (si >= 1) 
                    yd_i = this.x_f(1:this.n_dof);
                    dyd_i = this.x_f(this.n_dof+1:2*this.n_dof);
                    %Qi_ = 1e3*blkdiag( speye(n_dof,n_dof) , speye(n_dof,n_dof));
                end

%                 Phi = kron(this.I_ndof,phi); 
%                 Phi_dot = kron(this.I_ndof,phi_dot);
                
                Psi = sparse([kron(this.I_ndof,phi'); kron(this.I_ndof,phi_dot')]);
                xd_i = [yd_i; dyd_i];

                H = H + blkdiag(Psi'*Qi_*Psi, this.Q_slack);
                q = q - [Psi'*Qi_*xd_i; zeros(this.n_slack,1)];

                Aineq_i = sparse([kron(this.I_ndof,phi'); kron(this.I_ndof,phi_dot'); kron(this.I_ndof,phi_ddot')]);
                Aineq((i-1)*n_dof3+1 : i*n_dof3, :) = [Aineq_i, this.Aineq_slack];

            end
            
            this.log_.y_current = this.x0(1:this.n_dof);
            this.log_.dy_current = this.x0(this.n_dof+1:2*this.n_dof);
            this.log_.ddy_current = this.x0(2*this.n_dof+1:end);
            this.log_.si_data = si_data;
   
            Aineq(this.N*n_dof3+1 : this.N*n_dof3+this.N_obst, :) = A_obst;
            lb(this.N*n_dof3+1 : this.N*n_dof3+this.N_obst) = b_obst;
            
%             Aineq(this.N*n_dof3+1 : this.N*n_dof3+4, :) = A_obst(1:4,:);
%             lb(this.N*n_dof3+1 : this.N*n_dof3+4) = b_obst(1:4);


            H = (H+H')/2; % to account for numerical errors

            %% =======  initial state constraint  =======
            Aeq = [this.Phi0, sparse(n_dof3, this.n_slack)];
            beq = [this.x0];
            
            %% =======  final state constraint  =======
            Aineq = [Aineq; [this.Phi_f, sparse(n_dof3, this.n_slack)] ];
            lb = [lb; this.x_f - this.err_tol_f];
            ub = [ub; this.x_f + this.err_tol_f];

            % add final state to cost function too, to help the optimizer?
            % Maybe no... it could affect tracking during intermediate steps...
%             Psi = this.Phi_f;
%             xd_i = this.x_f;
%             Qi_ = blkdiag(1e3*speye(this.n_dof), 1e2*speye(this.n_dof), 1e1*speye(this.n_dof));
%             H = H + sparse( blkdiag(Psi'*Qi_*Psi, zeros(this.n_slack)) );
%             q = q - [Psi'*Qi_*xd_i; zeros(this.n_slack,1)];

            %% =======  process via-points  =======
            vp_c = this.process_via_points(s, s_dot);
            Aineq = [Aineq; vp_c.Aineq];
            lb = [lb; vp_c.lb];
            ub = [ub; vp_c.ub];
            H = H + vp_c.H;
            q = q + vp_c.q;

            
            %% =======  expand/shrink dual solution guess, in case constraints were added/removed =======
            n_ineq_plus = size(Aineq,1) - length(this.Z0_dual_ineq);
            if (n_ineq_plus>0), this.Z0_dual_ineq = [this.Z0_dual_ineq; zeros(n_ineq_plus, 1)]; 
            else, this.Z0_dual_ineq = this.Z0_dual_ineq(1:size(Aineq,1));
            end

%             A_sp = nnz(Aineq) / numel(Aineq);
%             H_sp = nnz(H) / numel(H);
%             
%             if (this.A_sp < A_sp)
%                 this.A_sp = A_sp;
%                 this.A_size = [nnz(Aineq) numel(Aineq)];
%             end
%             
%             if (this.H_sp < H_sp)
%                 this.H_sp = H_sp;
%                 this.H_size = [nnz(H) numel(H)];
%             end
            
            
            %% =======  solve optimization problem  =======

            A_osqp = [Aineq; Aeq];
            lb = [lb; beq];
            ub = [ub; beq];

            Z0_dual = [this.Z0_dual_ineq; this.Z0_dual_eq];
            
            % Create an OSQP object
            osqp_solver = osqp;
            osqp_solver.setup(H, q, A_osqp, lb, ub, 'warm_start',false, 'verbose',false, ...
                'eps_abs',this.settings.abs_tol, 'eps_rel',this.settings.rel_tol, ...
                'max_iter',this.settings.max_iter, 'time_limit',this.settings.time_limit);
            osqp_solver.warm_start('x', this.Z0, 'y',Z0_dual);

            res = osqp_solver.solve();

            solution.exit_flag = 0;
            if ( res.info.status_val ~= 1)
                %res.info
                solution.exit_msg = res.info.status;
                solution.exit_flag = 1;
                if (res.info.status_val == -3 || res.info.status_val == -4 || res.info.status_val == -7 || res.info.status_val == -10)
                    solution.exit_flag = -1;
                    return; 
                end
            end

            %% =======  extract the solution  =======
            Z = res.x;
            this.Z0 = Z;
            Z0_dual = res.y;
            n_ineq = size(Aineq,1);
            this.Z0_dual_ineq = Z0_dual(1:n_ineq);
            this.Z0_dual_eq = Z0_dual(n_ineq+1:end);
            
            w = Z(1:end-this.n_slack);
            slack_var = Z(end-this.n_slack+1:end);
            this.W_mpc = reshape(w, N_kernels, this.n_dof)';
            
%             obst_err = A_obst*Z - b_obst;
%             if (~isempty(find(obst_err < 0)))
%                 obst_err'
%             end

            if (~isempty(this.plot_callback)), this.plot_callback(this.log_); end

            %% =======  Generate optimal output  =======
            
            s_temp = this.can_sys_fun(s, s_dot);
            s_ddot = s_temp(2);
            
            solution.y = this.getYd(s);
            solution.y_dot = this.getYdDot(s, s_dot);
            solution.y_ddot = this.getYdDDot(s, s_dot, s_ddot);
            i1=1; i2=this.n_dof;
            if (this.pos_slack)
                solution.pos_slack = slack_var(i1:i2);
                i1 = i1 + this.n_dof;
                i2 = i2 + this.n_dof;
            end
            if (this.vel_slack)
                solution.vel_slack = slack_var(i1:i2);
                i1 = i1 + this.n_dof;
                i2 = i2 + this.n_dof;
            end
            if (this.accel_slack)
                solution.accel_slack = slack_var(i1:i2);
            end
            
            % check, because if the optimization finishes prematurely,
            % sometimes it can happen that the slacks are violated...
            % Not sure why this happens...?
            max_violation = max( abs(slack_var) - slack_lim );
%             if ( max_violation > 5e-4 )
%                 solution.exit_msg = ['Slack variable limits violated: max violation = ' num2str(max_violation)];
%                 solution.exit_flag = -1;
%                 return;
%             end
            
            %% =======  update initial state constraints  =======
            this.setInitialState(solution.y, solution.y_dot, solution.y_ddot, s, s_dot, s_ddot);
            
        end
        
        function yd = getYd(this, s)
           
            yd = this.W_mpc*this.gmp_mpc.regressVec(s);
            
        end
        
        function yd_dot = getYdDot(this, s, s_dot)
           
            yd_dot = this.W_mpc*this.gmp_mpc.regressVecDot(s, s_dot);
            
        end
        
        function yd_ddot = getYdDDot(this, s, s_dot, s_ddot)
           
            yd_ddot = this.W_mpc*this.gmp_mpc.regressVecDDot(s, s_dot, s_ddot);
            
        end
        
    end
    
    properties (Access = public)
        
        log_ = GMP_MPC_log()
        plot_callback = []

        settings % struct with settings
        
%         A_sp
%         A_size
%         
%         H_sp
%         H_size
        
    end
    
    %% ==================================
    %% ========== Protected =============
    %% ==================================

    
    properties (Access = protected)
        
        obj_shift_thres
            
        pos_gain
        vel_gain
        
        obstacles  % array with obstacles expressed as ellipsoids defined by their covariance and center
        N_obst
        
        n_var % number of optimization variables
        
        can_sys_fun
        
        via_points
        
        gmp_ref % const GMP*

        W_mpc % mpc optimized weights
        gmp_mpc % std::shared_ptr<GMP>

        N % unsigned
        dt_ % arma::rowvec(N)
        
        Aineq_slack % arma::mat(3*n_dof, n_slack)
        Q_slack % arma::mat(n_slack, n_slack)
        
        Qi % arma::mat(n_dof*N_kernels, n_dof*N_kernels)
        QN % arma::mat(n_dof*N_kernels, n_dof*N_kernels)
        
        Z0 
        Z0_dual_ineq
        Z0_dual_eq
        
        pos_lb % arma::vec(n_dof)
        pos_ub % arma::vec(n_dof)
        
        vel_lb % arma::vec(n_dof)
        vel_ub % arma::vec(n_dof)
        
        accel_lb % arma::vec(n_dof)
        accel_ub % arma::vec(n_dof)
        
        n_slack % unsigned
        
        n_dof % unsigned
        
        I_ndof % arma::mat().eye(n_dof,n_dof)
        
        pos_slack % bool
        vel_slack % bool
        accel_slack % bool
        
        pos_slack_lim % double
        vel_slack_lim % double
        accel_slack_lim % double
        
        Phi0 % arma::mat(3*n_dof, n_dof*N_kernels)
        x0 % arma::vec(3*n_dof)
        
        s_f % double
        Phi_f % arma::mat(3*n_dof, n_dof*N_kernels)
        x_f % arma::vec(3*n_dof)
        err_tol_f
        
    end
    
end