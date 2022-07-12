
classdef OnlinePlot < handle
    
    methods (Access = public)
        
        function this = OnlinePlot(dim, gmp_mpc)
        
            this.gmp_mpc = gmp_mpc;
            
            if (dim == 3)
                this.plot_ = @(varargin) plot3(varargin{1}(1,:), varargin{1}(2,:), varargin{1}(3,:), varargin{2:end});
                this.plot_obst_ = @(varargin) surf(varargin{1}{1}, varargin{1}{2}, varargin{1}{3}, varargin{2:end});
                this.plot_constr_ = @(varargin) patch('XData',varargin{1}{1}, 'YData',varargin{1}{2}, 'ZData',varargin{1}{3}, varargin{2:end});
                this.quiver_ = @(varargin) quiver3(varargin{1}(1), varargin{1}(2), varargin{1}(3), varargin{2}(1), varargin{2}(2), varargin{2}(3), varargin{3:end}); 
                this.color_ = 'FaceColor';
            elseif (dim == 2)
                this.plot_ = @(varargin) plot(varargin{1}(1,:), varargin{1}(2,:), varargin{2:end});
                this.plot_obst_ = @(varargin) plot(varargin{1}{1}, varargin{1}{2}, varargin{2:end});
                this.plot_constr_ = @(varargin) plot(varargin{1}{1}, varargin{1}{2}, varargin{2:end});
                this.quiver_ = @(varargin) quiver(varargin{1}(1), varargin{1}(2), varargin{2}(1), varargin{2}(2), varargin{3:end});
                this.color_ = 'Color';
            else
               error('Only 2 and 3 dim is supported.'); 
            end
            
            this.dim = dim;
            this.nan_ = nan(dim, 1);
        end
        
        function init(this, Pd_data, pos_lb, pos_ub, pos_slack, obst_curves, plot_every, view_)
            
            if (nargin < 7), plot_every = 5; end
            if (nargin < 8), view_ = [18, 17]; end

            this.plot_every = plot_every;
            
            obst_colors = {[1.0, 0.4, 0], [0.4, 0.2, 0], [0.4, 0.6, 0], [0.4, 0, 0.8], ...
                [0.2, 0.6, 0.4], [0.4, 0, 0.2], [0, 0.2, 0.4], [0.6, 0.2, 0.6], [0.4, 0.4, 0.6]};

            this.fig = figure;
            this.fig.Position(3:4) = [1112 844];
            this.ax = axes();
            hold(this.ax, 'on');
            % ----- create legend -----
            this.dmp_traj_properties = {'LineWidth',2.5, 'LineStyle',':', 'Parent',this.ax, 'color','blue'};
            this.opt_traj_properties = {'LineWidth',2.5, 'Color','magenta', 'Parent',this.ax};
            this.pred_traj_properties = {'LineWidth',2, 'Color',[1 0.7 1], 'LineStyle','-'};
            this.yd_properties = {'LineWidth',2, 'Marker','*', 'LineStyle','None', 'Markersize',14, 'Color','cyan', 'Parent',this.ax};
            this.y_pred_properties = {'LineWidth',2, 'Marker','o', 'LineStyle','None', 'Markersize',14, 'Color','magenta', 'Parent',this.ax};
            if (this.dim == 3)
                this.obst_constr_properties = {'FaceColor',[0 0.8 0], 'FaceAlpha',0.4, 'LineStyle','None', 'Parent',this.ax};
            else
                this.obst_constr_properties = {'Color',[0 0.8 0], 'LineStyle','-', 'LineWidth',2.3, 'Parent',this.ax};
            end
            
            this.plot_(this.nan_, this.dmp_traj_properties{:}, 'DisplayName','DMP traj');
            this.plot_(this.nan_, this.opt_traj_properties{:}, 'DisplayName','$\overline{DMP}^*$ traj');
            % this.plot_(nan, nan, nan,'LineWidth',2, 'Color',[1 0.7 1], 'LineStyle','-', 'Marker','*', 'MarkerSize',16, 'DisplayName','predicted $\overline{DMP}^*$ traj');
            this.plot_(this.nan_, this.pred_traj_properties{:}, 'DisplayName','predicted $\overline{DMP}^*$ traj');
            this.plot_(this.nan_, this.yd_properties{:}, 'DisplayName','predicted DMP points');
            this.plot_(this.nan_, this.y_pred_properties{:}, 'DisplayName','predicted $\overline{DMP}^*$ points');
            this.plot_constr_(num2cell(this.nan_), this.obst_constr_properties{:}, 'DisplayName','obst constraint');
            if (this.dim == 2)
                plot(nan, nan, 'LineWidth',2, 'Color',[1 0.5 0.5], 'LineStyle','--', 'DisplayName','soft limits');
                plot(nan, nan, 'LineWidth',2, 'Color',[0.5 0.5 0.5], 'LineStyle','--', 'DisplayName','hard limits');
                legend(this.ax, 'interpreter','latex', 'fontsize',14, 'Position',[0.7446 0.7129 0.2467 0.2573]);
                % ----- pos soft limits -----
                plot([pos_lb(1), pos_ub(1)], [pos_lb(2), pos_lb(2)], 'LineWidth',2, 'Color',[1 0.5 0.5], 'LineStyle','--', 'HandleVisibility','off');
                plot([pos_lb(1), pos_ub(1)], [pos_ub(2), pos_ub(2)], 'LineWidth',2, 'Color',[1 0.5 0.5], 'LineStyle','--', 'HandleVisibility','off');
                plot([pos_lb(1), pos_lb(1)], [pos_lb(2), pos_ub(2)], 'LineWidth',2, 'Color',[1 0.5 0.5], 'LineStyle','--', 'HandleVisibility','off');
                plot([pos_ub(1), pos_ub(1)], [pos_lb(2), pos_ub(2)], 'LineWidth',2, 'Color',[1 0.5 0.5], 'LineStyle','--', 'HandleVisibility','off');
                % ----- pos hard limits -----
                pos_hlb = pos_lb - pos_slack;
                pos_hub = pos_ub + pos_slack;
                plot([pos_hlb(1), pos_hub(1)], [pos_hlb(2), pos_hlb(2)], 'LineWidth',2, 'Color',[0.5 0.5 0.5], 'LineStyle','--', 'HandleVisibility','off');
                plot([pos_hlb(1), pos_hub(1)], [pos_hub(2), pos_hub(2)], 'LineWidth',2, 'Color',[0.5 0.5 0.5], 'LineStyle','--', 'HandleVisibility','off');
                plot([pos_hlb(1), pos_hlb(1)], [pos_hlb(2), pos_hub(2)], 'LineWidth',2, 'Color',[0.5 0.5 0.5], 'LineStyle','--', 'HandleVisibility','off');
                plot([pos_hub(1), pos_hub(1)], [pos_hlb(2), pos_hub(2)], 'LineWidth',2, 'Color',[0.5 0.5 0.5], 'LineStyle','--', 'HandleVisibility','off');
            end
            
            % this.plot_(nan, nan, 'LineWidth',2, 'Marker','x', 'LineStyle','None', 'Markersize',14, 'Color','red', 'Parent',this.ax, 'DisplayName','obst constr trigger');
            % this.plot_(nan, nan, nan,'LineWidth',2, 'Color',[1 0.5 0.5], 'LineStyle','--', 'DisplayName','pos bound constr');
            legend(this.ax, 'interpreter','latex', 'fontsize',14, 'Position',[0.7446 0.7129 0.2467 0.2573]);
            % ----- pos bounds -----
%             this.plot_([pos_lb(1), pos_ub(1)], [pos_lb(2), pos_lb(2)], 'LineWidth',2, 'Color',[1 0.5 0.5], 'LineStyle','--', 'HandleVisibility','off');
%             this.plot_(q_p[pos_lb(1), pos_ub(1)], [pos_ub(2), pos_ub(2)], 'LineWidth',2, 'Color',[1 0.5 0.5], 'LineStyle','--', 'HandleVisibility','off');
%             this.plot_([pos_lb(1), pos_lb(1)], [pos_lb(2), pos_ub(2)], 'LineWidth',2, 'Color',[1 0.5 0.5], 'LineStyle','--', 'HandleVisibility','off');
%             this.plot_([pos_ub(1), pos_ub(1)], [pos_lb(2), pos_ub(2)], 'LineWidth',2, 'Color',[1 0.5 0.5], 'LineStyle','--', 'HandleVisibility','off');
            % ----- Unconstrained trajectory --------
            this.plot_(Pd_data, this.dmp_traj_properties{:}, 'HandleVisibility','off');
            this.plot_(Pd_data(:, 1), 'LineWidth',2, 'LineStyle','None', 'Marker','o', 'Color',[0 0.8 0], 'MarkerSize',14, 'Parent',this.ax, 'HandleVisibility','off');
            this.plot_(Pd_data(:, end), 'LineWidth',2, 'LineStyle','None', 'Marker','x', 'Color',[0.8 0 0], 'MarkerSize',14, 'Parent',this.ax, 'HandleVisibility','off');
            % ----- Online generated trajectory -----
            this.p_h = this.plot_(this.nan_, this.opt_traj_properties{:}, 'HandleVisibility','off');
            % ----- Obstacles ------
            for i=1:length(obst_curves)
                color = obst_colors{mod(i, length(obst_colors))};
                Ep = obst_curves{i};
                h = this.plot_obst_(Ep, this.color_,color, 'LineWidth',2, 'Parent',this.ax, 'HandleVisibility','off');
                if (this.dim == 3), set(h, 'FaceAlpha',0.5, 'LineStyle','None'); end
                %this.plot_(c(1), c(2), 'LineWidth',2, 'Marker','x', 'LineStyle','None', 'Markersize',14, 'Color',color, 'Parent',this.ax, 'HandleVisibility','off');
            end
            axis(this.ax, 'tight');
            axis(this.ax, 'equal');
            this.ax.XLim = this.ax.XLim + 0.05*[-1, 1];
            this.ax.YLim = this.ax.YLim + 0.05*[-1, 1];
            xlabel('X [m]', 'fontsize',14, 'Parent',this.ax);
            ylabel('Y [m]', 'fontsize',14, 'Parent',this.ax);
            
            if (this.dim == 3)
                this.ax.ZLim = this.ax.ZLim + 0.05*[-1, 1];
                zlabel('Z [m]', 'fontsize',14, 'Parent',this.ax);
                view(this.ax, view_);
                grid(this.ax, 'on');
            end

        end

        function update(this, log_)

            this.p_h.XData = [this.p_h.XData log_.y_current(1)];
            this.p_h.YData = [this.p_h.YData log_.y_current(2)];
            if (this.dim == 3)
                this.p_h.ZData = [this.p_h.ZData log_.y_current(3)];
            end
            
            this.plt_show_count = this.plt_show_count + 1;
            if (this.plt_show_count < this.plot_every), return; end

            delete(this.plt_handles);
            
            n_dof = length(log_.y_current);
            
            %s_next = linspace(log_.si_data(1), log_.si_data(end), 30);
            s_next = log_.si_data;
            P_next = zeros(n_dof, length(s_next));
            for j=1:length(s_next), P_next(:,j) = this.gmp_mpc.getYd(s_next(j)); end
            %h = this.plot_(P_next(1,:), P_next(2,:), P_next(3,:), 'LineWidth',2, 'Color',[1 0.7 1], 'LineStyle','None', 'Marker','*', 'MarkerSize',16, 'HandleVisibility','off');
            %this.plt_handles = [this.plt_handles h];
            s_next = linspace(log_.si_data(1), log_.si_data(end), 30);
            P_next = zeros(n_dof, length(s_next));
            for j=1:length(s_next), P_next(:,j) = this.gmp_mpc.getYd(s_next(j)); end
            h = this.plot_(P_next, this.pred_traj_properties{:}, 'HandleVisibility','off');
            this.plt_handles = [this.plt_handles h];

            for i=1:length(log_.n_e_data)
                c = log_.c_data{i};
                n_e = log_.n_e_data{i};
                p1 = log_.p_data{i};
                p_e = log_.p_e_data{i};

                obst_constr_points = OnlinePlot.createObstConstr(p_e, n_e, 0.1);

                h1 = this.quiver_(p_e, n_e, 0.08, 'Color','green', 'LineWidth',3, 'Parent',this.ax, 'HandleVisibility','off');
                h2 = this.plot_constr_(obst_constr_points, this.obst_constr_properties{:}, 'HandleVisibility','off');
                h3 = this.plot_([c p_e], 'LineWidth',2, 'LineStyle',':', 'Color','red', 'Parent',this.ax, 'HandleVisibility','off');
                h4 = this.plot_(p1, 'LineWidth',2, 'Marker','x', 'LineStyle','None', 'Markersize',14, 'Color','red', 'Parent',this.ax, 'HandleVisibility','off');
                % h5 = this.plot_(p2(1), p2(2), 'LineWidth',2, 'Marker','x', 'LineStyle','None', 'Markersize',14, 'Color','magenta', 'Parent',this.ax, 'HandleVisibility','off');
                % axis(this.ax, 'equal');

                this.plt_handles = [this.plt_handles h1 h2 h3 h4];
            end
        
            h = this.plot_(log_.yd_points, this.yd_properties{:}, 'HandleVisibility','off');
            this.plt_handles = [this.plt_handles h];
            
            h = this.plot_(log_.y_pred_points, this.y_pred_properties{:}, 'HandleVisibility','off');
            this.plt_handles = [this.plt_handles h];

            drawnow();
            this.plt_show_count = 0;
            pause(0.001);

        end
        
    end
    
    methods (Static, Access = protected)
       
        function points = createObstConstr(center, normal, width)
            
            if (length(center) == 3)
                % Ax + By + Cz + D = 0
                A = normal(1);
                B = normal(2);
                C = normal(3);
                D = -dot(normal, center);

                % calc plane vertices
                X = center(1) + [-1  1 1 -1]; % Generate data for x vertices
                Y = center(2) + [-1 -1 1  1]; % Generate data for y vertices
                Z = -1/C*(A*X + B*Y + D); % Solve for z vertices data

                % calc scaling so all plane edges have unit length
                P = [X; Y; Z];
                sx = norm(P(:,1)-P(:,2));
                sy = norm(P(:,2)-P(:,3));

                % recalculate plane vertices with the appropriate scaling
                X = center(1) + width*[-1  1 1 -1]/sx;
                Y = center(2) + width*[-1 -1 1  1]/sy;
                Z = -1/C*(A*X + B*Y + D);

                points = {X, Y, Z};
            else
                v = [normal(2); - normal(1)]; % tangent line direction
                p1 = center - 0.5*width*v;
                p2 = center + 0.5*width*v;
                X = [p1(1), p2(1)];
                Y = [p1(2), p2(2)];
                points = {X, Y};
            end

        end
        
    end

    properties (Access = protected)
        
        color_
        
        dim
        
        plot_
        plot_constr_
        plot_obst_
        quiver_
        
        nan_
        
        gmp_mpc
        plot_every = 10
        fig
        ax
        p_h % handle for the path executed so far
        plt_handles = []
        plt_show_count = 0
        
        dmp_traj_properties
        opt_traj_properties
        pred_traj_properties
        yd_properties
        y_pred_properties
        obst_constr_properties
        
    end
    
    
end