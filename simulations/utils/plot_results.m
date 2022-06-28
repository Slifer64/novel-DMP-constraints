function plot_results(data, y0, yg, ygd, tau, pos_lim, vel_lim, accel_lim, via_points, view_, plot_all_traj_same_fig, select_bounds_to_plot)
%% ======== Plot Results ==========

if (nargin < 9), via_points = []; end
if (nargin < 10), view_ = []; end
if (nargin < 11), plot_all_traj_same_fig = false; end
if (nargin < 12), select_bounds_to_plot = [1; 1; 1]; end

n_dof = length(y0);

t_vp = [];
vp_mat = [];
for i=1:length(via_points)
    t_vp = [t_vp via_points{i}.s*tau];
    vp_mat = [vp_mat via_points{i}.pos]; 
end

global slack_limits

p_slack = slack_limits(1);
v_slack = slack_limits(2);
a_slack = slack_limits(3);

%% --------- plot 3D paths ----------
if (n_dof == 3)
    fig = figure;
    fig.Position(3:4) = [815 716];
    ax = axes();
    hold on;
    plot3(y0(1), y0(2), y0(3), 'LineWidth', 4, 'LineStyle','none', 'Color',[0.47,0.67,0.19],'Marker','o', 'MarkerSize',12, 'DisplayName', '$p_0$');
    plot3(yg(1), yg(2), yg(3), 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',12, 'DisplayName', '$g$');
    plot3(ygd(1), ygd(2), ygd(3), 'LineWidth', 4, 'LineStyle','none', 'Color','magenta','Marker','x', 'MarkerSize',12, 'DisplayName', '$g_d$');
    if (~isempty(via_points))
        plot3(vp_mat(1,:), vp_mat(2,:), vp_mat(3,:), 'LineWidth', 3, 'LineStyle','none', 'Color','red','Marker','*', 'MarkerSize',12, 'DisplayName','via-points');
    end
    legend_ = {};
    for k=1:length(data)
        if (~data{k}.plot3D), continue; end
        plot3(data{k}.Pos(1,:), data{k}.Pos(2,:), data{k}.Pos(3,:), 'LineWidth', 3, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color, 'DisplayName',data{k}.legend);
        %legend_ = [legend_ data{k}.legend];
    end
    plot3([ygd(1) yg(1)], [ygd(2) yg(2)], [ygd(3) yg(3)], 'LineWidth', 1, 'LineStyle','--', 'Color',[1 0 1 0.5], 'HandleVisibility','off');
    legend({}, 'interpreter','latex', 'fontsize',17, 'Position',[0.8174 0.6641 0.1531 0.3144]);
%     axis tight;
%     x_lim = ax.XLim + 0.05*[-1 1];
%     y_lim = ax.YLim + 0.05*[-1 1];
%     z_lim = ax.ZLim + 0.05*[-1 1];
    plot3Dbounds(ax, pos_lim + p_slack*[-1 1], select_bounds_to_plot);
    if (~isempty(view_)), view(view_); 
    else, view(-164, 19.4);
    end
    grid on;
%     ax.XLim=x_lim; ax.YLim=y_lim; ax.ZLim=z_lim;
    ax.FontSize = 14;
    xlabel('x [$m$]', 'interpreter','latex', 'fontsize',18);
    ylabel('y [$m$]', 'interpreter','latex', 'fontsize',18);
    zlabel('z [$m$]', 'interpreter','latex', 'fontsize',18);
    title('Cartesian path', 'interpreter','latex', 'fontsize',20);
    hold off;
end

%% ------- Plot trajectories ---------


if (n_dof == 3)
    title_ = {'x axis', 'y axis', 'z axis'};
else
    title_ = cell(n_dof,1);
    for i=1:n_dof, title_{i} = ['DoF' num2str(i)]; end
end

title_font = 20;
label_font = 17;
ax_fontsize = 14;

axes_handles = cell(3,3);

if (plot_all_traj_same_fig)
    fig = figure;
    fig.Position(3:4) = [842 1110];
    k = 1;
    for i=1:3 % pos, vel, aceel
        for j=1:n_dof
            axes_handles{i,j} = subplot(3,n_dof,k);
            k = k+1;
        end
    end
else
    for j=1:n_dof
        fig = figure;
        fig.Position(3:4) = [842 1110];
        for i=1:3
            axes_handles{i,j} = subplot(3,1,i);
        end
    end
end

for i=1:n_dof
    % ======== position ==========
    
    ax = axes_handles{1,i};
    axes(ax); % set this axis as the current axes
    ax_vec = [ax];
    hold on;
    % plot position trajectory
    legend_ = {};
    for k=1:length(data)
        if (~data{k}.plot2D), continue; end
        plot(data{k}.Time, data{k}.Pos(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color);
        legend_ = [legend_ data{k}.legend];
    end
    axis tight;
    % plot start and final positions
    plot(0, y0(i), 'LineWidth', 4, 'LineStyle','none', 'Color',[0.47,0.67,0.19],'Marker','o', 'MarkerSize',10);
    plot(tau, yg(i), 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10);
    plot(tau, ygd(i), 'LineWidth', 4, 'LineStyle','none', 'Color','magenta','Marker','x', 'MarkerSize',10);
    if (~isempty(via_points))
        plot(t_vp, vp_mat(i,:), 'LineWidth', 3, 'LineStyle','none', 'Color','red','Marker','*', 'MarkerSize',10, 'HandleVisibility','off');
    end
    % plot soft limits
    plot(ax.XLim, [pos_lim(i,1) pos_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
    plot(ax.XLim, [pos_lim(i,2) pos_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
    % plot hard limits
    plot(ax.XLim, [pos_lim(i,1) pos_lim(i,1)]-p_slack, 'LineWidth',2, 'LineStyle','--', 'Color',[0.5 0.5 0.5], 'HandleVisibility','off');
    plot(ax.XLim, [pos_lim(i,2) pos_lim(i,2)]+p_slack, 'LineWidth',2, 'LineStyle','--', 'Color',[0.5 0.5 0.5], 'HandleVisibility','off');
    
    ax.FontSize = ax_fontsize;
    % labels, title ...
    ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',label_font);
    title(title_{i}, 'interpreter','latex', 'fontsize',title_font);
    legend(legend_, 'interpreter','latex', 'fontsize',17, 'Position',[0.2656 0.9598 0.4868 0.0344], 'Orientation', 'horizontal');
    hold off;
    
    % ======== velocity ==========
    ax = axes_handles{2,i};
    axes(ax); % set this axis as the current axes
    ax_vec = [ax_vec ax];
    hold on;
    plot(tau, 0, 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10);
    for k=1:length(data)
        if (~data{k}.plot2D), continue; end
        plot(data{k}.Time, data{k}.Vel(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color);
    end
    axis tight;
    % plot soft limits
    plot(ax.XLim, [vel_lim(i,1) vel_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
    plot(ax.XLim, [vel_lim(i,2) vel_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
    % plot hard limits
    plot(ax.XLim, [vel_lim(i,1) vel_lim(i,1)]-v_slack, 'LineWidth',2, 'LineStyle','--', 'Color',[0.5 0.5 0.5], 'HandleVisibility','off');
    plot(ax.XLim, [vel_lim(i,2) vel_lim(i,2)]+v_slack, 'LineWidth',2, 'LineStyle','--', 'Color',[0.5 0.5 0.5], 'HandleVisibility','off');

    ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',label_font);
    ax.FontSize = ax_fontsize;
    hold off;
    
    % ======== acceleration ==========
    ax = axes_handles{3,i};
    axes(ax); % set this axis as the current axes
    ax_vec = [ax_vec ax];
    hold on;
    plot(tau, 0, 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10);
    for k=1:length(data)
        if (~data{k}.plot2D), continue; end
        plot(data{k}.Time, data{k}.Accel(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color);
    end
    axis tight;
    % plot soft limits
    plot(ax.XLim, [accel_lim(i,1) accel_lim(i,1)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
    plot(ax.XLim, [accel_lim(i,2) accel_lim(i,2)], 'LineWidth',2, 'LineStyle','--', 'Color',[1 0 1]);
    % plot hard limits
    plot(ax.XLim, [accel_lim(i,1) accel_lim(i,1)]-a_slack, 'LineWidth',2, 'LineStyle','--', 'Color',[0.5 0.5 0.5], 'HandleVisibility','off');
    plot(ax.XLim, [accel_lim(i,2) accel_lim(i,2)]+a_slack, 'LineWidth',2, 'LineStyle','--', 'Color',[0.5 0.5 0.5], 'HandleVisibility','off');
    %ax.YLim = [ max(ax.YLim(1), 8*accel_lim(i,1)) min(ax.YLim(2), 8*accel_lim(i,2)) ];
    ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',label_font);
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',label_font);
    ax.FontSize = ax_fontsize;
    hold off;

    linkaxes(ax_vec,'x');

end


end

% ============  Utility functions =================

% param[in] ax: axes handle in which to plot the bounds.
% param[in] bounds: 3x2 matrix where the each row has the [lower upper]
%                   bounds for x,y,z respectively.
% param[in] select_bounds: 3x1 vector where the i-th entry is 1 to plot and
%                          0 to skip the bounds for the i-th DoF. (optional, default = [1;1;1])
function plot3Dbounds(ax, bounds, select_bounds)

    if (nargin < 3), select_bounds = [1; 1; 1]; end
    
    if (length(select_bounds) ~= 3), error('select_bounds must have length == 3'); end
    
    x1 = bounds(1,1);   x2 = bounds(1,2);
    y1 = bounds(2,1);   y2 = bounds(2,2);
    z1 = bounds(3,1);   z2 = bounds(3,2);

    X_x=[]; X_y=[]; X_z=[];
    if (select_bounds(1))
        %         x_min        x_max  
        X_x = [x1 x1 x1 x1; x2 x2 x2 x2];
        X_y = [y1 y1 y2 y2; y1 y1 y2 y2];
        X_z = [z1 z2 z2 z1; z1 z2 z2 z1];
    end

    Y_x=[]; Y_y=[]; Y_z=[];
    if (select_bounds(2))
        %         y_min        y_max  
        Y_x = [x1 x1 x2 x2; x1 x1 x2 x2];
        Y_y = [y1 y1 y1 y1; y2 y2 y2 y2];
        Y_z = [z1 z2 z2 z1; z1 z2 z2 z1];
    end
    
    Z_x=[]; Z_y=[]; Z_z=[];
    if (select_bounds(3))
        %         y_min        y_max  
        Z_x = [x1 x1 x2 x2; x1 x1 x2 x2];
        Z_y = [y1 y2 y2 y1; y1 y2 y2 y1];
        Z_z = [z1 z1 z1 z1; z2 z2 z2 z2];
    end
   
    X = [X_x; Y_x; Z_x]';
    Y = [X_y; Y_y; Z_y]';
    Z = [X_z; Y_z; Z_z]';

    patch(X, Y, Z, 'red', 'FaceAlpha',0.05, 'LineStyle','none', 'Parent',ax, 'HandleVisibility','off');

end

