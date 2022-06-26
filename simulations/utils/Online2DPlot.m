
classdef Online2DPlot < handle
    
    methods (Access = public)
        
        function this = Online2DPlot(gmp_mpc)
        
            this.gmp_mpc = gmp_mpc;
            
        end
        
        function init(this, Pd_data, pos_lb, pos_ub, obst_curves, plot_every)

            this.plot_every = plot_every;
            
            obst_colors = {[1.0, 0.4, 0], [0.4, 0.2, 0], [0.4, 0.6, 0], [0.4, 0, 0.8], ...
                [0.2, 0.6, 0.4], [0.4, 0, 0.2], [0, 0.2, 0.4], [0.6, 0.2, 0.6], [0.4, 0.4, 0.6]};

            this.fig = figure;
            this.fig.Position(3:4) = [1112 844];
            this.ax = axes();
            hold(this.ax, 'on');
            % ----- create legend -----
            plot(nan, nan, 'LineWidth',2, 'LineStyle',':', 'Parent',this.ax, 'color','blue', 'DisplayName','DMP traj');
            plot(nan, nan, 'LineWidth',2, 'Color','magenta', 'Parent',this.ax, 'DisplayName','$\overline{DMP}^*$ traj');
            plot(nan, nan, 'LineWidth',2, 'Color',[1 0.7 1], 'LineStyle','-', 'Marker','*', 'MarkerSize',16, 'DisplayName','predicted $\overline{DMP}^*$ traj');
            plot(nan, nan, 'LineWidth',2, 'Marker','x', 'LineStyle','None', 'Markersize',14, 'Color','cyan', 'Parent',this.ax, 'DisplayName','predicted DMP points');
            plot(nan, nan, 'LineWidth',2, 'Marker','o', 'LineStyle','None', 'Markersize',14, 'Color','magenta', 'Parent',this.ax, 'DisplayName','predicted $\overline{DMP}^*$ points');
            plot(nan, nan, 'LineWidth',2.2, 'LineStyle','-.', 'Color',[0 0.8 0], 'Parent',this.ax, 'DisplayName','obst constr plane');
            % plot(nan, nan, 'LineWidth',2, 'Marker','x', 'LineStyle','None', 'Markersize',14, 'Color','red', 'Parent',this.ax, 'DisplayName','obst constr trigger');
            plot(nan, nan, 'LineWidth',2, 'Color',[1 0.5 0.5], 'LineStyle','--', 'DisplayName','pos bound constr');
            legend(this.ax, 'interpreter','latex', 'fontsize',14, 'Position',[0.7446 0.7129 0.2467 0.2573]);
            % ----- pos bounds -----
            plot([pos_lb(1), pos_ub(1)], [pos_lb(2), pos_lb(2)], 'LineWidth',2, 'Color',[1 0.5 0.5], 'LineStyle','--', 'HandleVisibility','off');
            plot([pos_lb(1), pos_ub(1)], [pos_ub(2), pos_ub(2)], 'LineWidth',2, 'Color',[1 0.5 0.5], 'LineStyle','--', 'HandleVisibility','off');
            plot([pos_lb(1), pos_lb(1)], [pos_lb(2), pos_ub(2)], 'LineWidth',2, 'Color',[1 0.5 0.5], 'LineStyle','--', 'HandleVisibility','off');
            plot([pos_ub(1), pos_ub(1)], [pos_lb(2), pos_ub(2)], 'LineWidth',2, 'Color',[1 0.5 0.5], 'LineStyle','--', 'HandleVisibility','off');
            % ----- Unconstrained trajectory --------
            plot(Pd_data(1, :), Pd_data(2, :), 'LineWidth',2, 'LineStyle',':', 'Parent',this.ax, 'color','blue', 'HandleVisibility','off');
            plot(Pd_data(1, 1), Pd_data(2, 1), 'LineWidth',2, 'Marker','o', 'Color',[0 0.8 0], 'MarkerSize',14, 'Parent',this.ax, 'HandleVisibility','off');
            plot(Pd_data(1, end), Pd_data(2, end), 'LineWidth',2, 'Marker','x', 'Color',[0.8 0 0], 'MarkerSize',14, 'Parent',this.ax, 'HandleVisibility','off');
            % ----- Online generated trajectory -----
            this.p_h = plot(nan, nan, 'LineWidth',2, 'Color','magenta', 'Parent',this.ax, 'HandleVisibility','off');
            % ----- Obstacles ------
            for i=1:length(obst_curves)
                color = obst_colors{mod(i, length(obst_colors))};
                Ep = obst_curves{i};
                plot(Ep(1,:), Ep(2,:), 'LineWidth',2, 'Color',color, 'Parent',this.ax, 'HandleVisibility','off');
                %plot(c(1), c(2), 'LineWidth',2, 'Marker','x', 'LineStyle','None', 'Markersize',14, 'Color',color, 'Parent',this.ax, 'HandleVisibility','off');
            end
            axis(this.ax, 'tight');
            axis(this.ax, 'equal');
            this.ax.XLim = this.ax.XLim + [-0.03, 0.03];
            this.ax.YLim = this.ax.YLim + [-0.03, 0.03];
            xlabel('X [m]', 'fontsize',14, 'Parent',this.ax);
            ylabel('Y [m]', 'fontsize',14, 'Parent',this.ax);

        end

        function update_plot(this, log_)

            this.p_h.XData = [this.p_h.XData log_.y_current(1)];
            this.p_h.YData = [this.p_h.YData log_.y_current(2)];

            this.plt_show_count = this.plt_show_count + 1;
            if (this.plt_show_count < this.plot_every), return; end

            delete(this.plt_handles);
            
            n_dof = length(log_.y_current);

            %s_next = linspace(log_.si_data(1), log_.si_data(end), 30);
            s_next = log_.si_data;
            P_next = zeros(n_dof, length(s_next));
            for j=1:length(s_next), P_next(:,j) = this.gmp_mpc.getYd(s_next(j)); end
            h = plot(P_next(1,:), P_next(2,:), 'LineWidth',2, 'Color',[1 0.7 1], 'LineStyle','None', 'Marker','*', 'MarkerSize',16, 'HandleVisibility','off');
            this.plt_handles = [this.plt_handles h];
            s_next = linspace(log_.si_data(1), log_.si_data(end), 30);
            P_next = zeros(n_dof, length(s_next));
            for j=1:length(s_next), P_next(:,j) = this.gmp_mpc.getYd(s_next(j)); end
            h = plot(P_next(1,:), P_next(2,:), 'LineWidth',2, 'Color',[1 0.6 1], 'HandleVisibility','off');
            this.plt_handles = [this.plt_handles h];

            for i=1:length(log_.n_e_data)
                c = log_.c_data{i};
                n_e = log_.n_e_data{i};
                p1 = log_.p_data{i};
                p2 = log_.p_e_data{i};

                v = [n_e(2); - n_e(1)]; % tangent line direction
                p3 = p2 - 0.1*v;
                p4 = p2 + 0.1*v;

                h1 = quiver(p2(1), p2(2), n_e(1), n_e(2), 0.08, 'Color','green', 'LineWidth',3, 'Parent',this.ax, 'HandleVisibility','off');
                h2 = plot([p3(1) p4(1)], [p3(2) p4(2)], 'LineWidth',2.2, 'LineStyle','-.', 'Color',[0 0.8 0], 'Parent',this.ax, 'HandleVisibility','off');
                h3 = plot([c(1) p2(1)], [c(2) p2(2)], 'LineWidth',2, 'LineStyle',':', 'Color','red', 'Parent',this.ax, 'HandleVisibility','off');
                h4 = plot(p1(1), p1(2), 'LineWidth',2, 'Marker','x', 'LineStyle','None', 'Markersize',14, 'Color','red', 'Parent',this.ax, 'HandleVisibility','off');
                % h5 = plot(p2(1), p2(2), 'LineWidth',2, 'Marker','x', 'LineStyle','None', 'Markersize',14, 'Color','magenta', 'Parent',this.ax, 'HandleVisibility','off');
                % axis(this.ax, 'equal');

                this.plt_handles = [this.plt_handles h1 h2 h3 h4];
            end

            for j=1:size(log_.yd_points, 2)
                h = plot(log_.yd_points(1,j), log_.yd_points(2,j), 'LineWidth',2, 'Marker','x', 'LineStyle','None', 'Markersize',14, 'Color','cyan', 'Parent',this.ax, 'HandleVisibility','off');
                this.plt_handles = [this.plt_handles h];
            end

            for j=1:size(log_.y_pred_points, 2)
                h = plot(log_.y_pred_points(1,j), log_.y_pred_points(2,j), 'LineWidth',2, 'Marker','o', 'LineStyle','None', 'Markersize',14, 'Color','magenta', 'Parent',this.ax, 'HandleVisibility','off');
                this.plt_handles = [this.plt_handles h];
            end

            drawnow();
            this.plt_show_count = 0;
            pause(0.001);

        end
        
    end

    properties (Access = protected)
        
        gmp_mpc
        plot_every = 10
        fig
        ax
        p_h % handle for the path executed so far
        plt_handles = []
        plt_show_count = 0
        
    end
    
    
end