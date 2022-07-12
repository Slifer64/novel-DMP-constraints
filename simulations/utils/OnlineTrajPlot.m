
classdef OnlineTrajPlot < handle
    
    methods (Access = public)
        
        function this = OnlineTrajPlot(pos_lim, vel_lim, accel_lim, slack_lim, Time, Pd_data, dPd_data, ddPd_data)
            
            y0 = Pd_data(:,1);
            yg = Pd_data(:,end);
            Tf = Time(end);
            
            pos_hlim = pos_lim + slack_lim(1)*[-1, 1];
            vel_hlim = vel_lim + slack_lim(2)*[-1, 1];
            accel_hlim = accel_lim + slack_lim(3)*[-1, 1];
            
            ref_traj_properties = {'LineWidth',2, 'LineStyle','--', 'Color', 'blue'};
            opt_traj_properties = {'LineWidth',2, 'LineStyle','-', 'Color', 'magenta'};
            solf_lim_properties = {'LineWidth',2, 'LineStyle','--', 'Color',[1 0.5 0.5]};
            hard_lim_properties = {'LineWidth',2, 'LineStyle','--', 'Color',[0.5 0.5 0.5]};
            
            title_font = 20;
            label_font = 18;
            axis_font = 20;
            
            n_dof = length(y0);
            
            ind = 1:n_dof; % choose DoFs to plot, e.g. [1 2 3] for [x y z]
            
            this.ax_ = cell(n_dof, length(ind));
            this.lh_ = cell(n_dof, length(ind));
            
            titles = {'X-axis', 'Y-axis', 'Z-axis'};
            
            fig = figure;
            fig.Position(3:4) = [1762 850];

            %this.fig_{k} = fig;
            this.fig_ = fig;
              
            if (n_dof == 3), count = [1 4 7];
            else, count = [1 3 5];
            end
            
            for k=1:length(ind)

                i = ind(k);
                % ----- position trajectory ------
                ax = subplot(3,n_dof,count(1), 'Parent',fig);
                this.ax_{k, 1} = ax;
                hold(ax, 'on');
                plot(Time, Pd_data(i,:), ref_traj_properties{:}, 'Parent',ax);
                this.lh_{k, 1} = plot(nan, nan, opt_traj_properties{:}, 'Parent',ax);
                % plot start and final positions
                plot(0, y0(i), 'LineWidth', 4, 'LineStyle','none', 'Color','green','Marker','o', 'MarkerSize',10, 'Parent',ax);
                plot(Tf, yg(i), 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10, 'Parent',ax);
                % plot bounds
                plot(ax.XLim, [pos_lim(i,1) pos_lim(i,1)], solf_lim_properties{:}, 'Parent',ax);
                plot(ax.XLim, [pos_lim(i,2) pos_lim(i,2)], solf_lim_properties{:}, 'Parent',ax);
                % plot hard limits
                plot(ax.XLim, [pos_hlim(i,1) pos_hlim(i,1)], hard_lim_properties{:}, 'Parent',ax);
                plot(ax.XLim, [pos_hlim(i,2) pos_hlim(i,2)], hard_lim_properties{:}, 'Parent',ax);
                % labels, title ...
                ax.FontSize = axis_font;
                title(ax, titles{i}, 'interpreter','latex', 'fontsize',title_font);
                % ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',ax);
                % legend(legend_, 'interpreter','latex', 'fontsize',17, 'Position',[0.2330 0.9345 0.5520 0.0294], 'Orientation', 'horizontal');
                axis(ax, 'tight');
                axis(ax, 'manual');
                
                % ----- velocity trajectory ------
                ax = subplot(3,n_dof,count(2), 'Parent',fig);
                this.ax_{k, 2} = ax;
                hold(ax, 'on');
                plot(Time, dPd_data(i,:), ref_traj_properties{:}, 'Parent',ax);
                this.lh_{k, 2} = plot(nan, nan, opt_traj_properties{:}, 'Parent',ax);
                % plot bounds
                plot(ax.XLim, [vel_lim(i,1) vel_lim(i,1)], solf_lim_properties{:}, 'Parent',ax);
                plot(ax.XLim, [vel_lim(i,2) vel_lim(i,2)], solf_lim_properties{:}, 'Parent',ax);
                % plot hard limits
                plot(ax.XLim, [vel_hlim(i,1) vel_hlim(i,1)], hard_lim_properties{:}, 'Parent',ax);
                plot(ax.XLim, [vel_hlim(i,2) vel_hlim(i,2)], hard_lim_properties{:}, 'Parent',ax);
                ax.FontSize = axis_font;
                % ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',ax);
                axis(ax, 'tight');
                axis(ax, 'manual');

                % ----- acceleration trajectory ------
                ax = subplot(3,n_dof,count(3));
                this.ax_{k, 3} = ax;
                hold(ax, 'on');
                plot(Time, ddPd_data(i,:), ref_traj_properties{:}, 'Parent',ax);
                this.lh_{k, 3} = plot(nan, nan, opt_traj_properties{:}, 'Parent',ax);
                % plot bounds
                plot(ax.XLim, [accel_lim(i,1) accel_lim(i,1)], solf_lim_properties{:}, 'Parent',ax);
                plot(ax.XLim, [accel_lim(i,2) accel_lim(i,2)], solf_lim_properties{:}, 'Parent',ax);
                % plot hard limits
                plot(ax.XLim, [accel_hlim(i,1) accel_hlim(i,1)], hard_lim_properties{:}, 'Parent',ax);
                plot(ax.XLim, [accel_hlim(i,2) accel_hlim(i,2)], hard_lim_properties{:}, 'Parent',ax);
                ax.FontSize = axis_font;
                % ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',ax);
                xlabel('time [$s$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',ax);
                axis(ax, 'tight');
                axis(ax, 'manual');
                
                count = count + 1;

            end
            
            ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',this.ax_{1, 1});
            ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',this.ax_{1, 2});
            ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',this.ax_{1, 3});
            
        end
        
        function update(this, t, y, y_dot, y_ddot)

            for i=1:length(y)
                h = this.lh_{i, 1};
                h.XData = [h.XData t];
                h.YData = [h.YData y(i)];
                
                h = this.lh_{i, 2};
                h.XData = [h.XData t];
                h.YData = [h.YData y_dot(i)];
                
                h = this.lh_{i, 3};
                h.XData = [h.XData t];
                h.YData = [h.YData y_ddot(i)];
            end
            
            this.plt_show_count = this.plt_show_count + 1;
            if (this.plt_show_count < this.plot_every), return; end

            drawnow();
            this.plt_show_count = 0;
            pause(0.001);

        end
        
    end
    

    properties (Access = protected)

        plot_every = 10
        fig_
        ax_
        lh_
        plt_show_count = 0
        
    end
    
    
end