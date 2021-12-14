function plotSlackVariables(Time, pos_slack_data, vel_slack_data, accel_slack_data, slack_limits)

    pos_slack = ~isempty(pos_slack_data);
    vel_slack = ~isempty(vel_slack_data);
    accel_slack = ~isempty(accel_slack_data);
    n_slack = pos_slack + vel_slack + accel_slack;
    if (n_slack)

        slack_labels = {};
        slack_lim = [];
        slack_data = {};
        if (pos_slack)
            slack_lim = [slack_lim slack_limits(1)];
            slack_labels = [slack_labels {'pos'}];
            slack_data = [slack_data {pos_slack_data}];
        end
        if (vel_slack)
            slack_lim = [slack_lim slack_limits(2)];
            slack_labels = [slack_labels {'vel'}];
            slack_data = [slack_data {vel_slack_data}];
        end
        if (accel_slack)
            slack_lim = [slack_lim slack_limits(3)];
            slack_labels = [slack_labels {'accel'}];
            slack_data = [slack_data {accel_slack_data}];
        end
        
        n_dof = size(slack_data{1},1);

        figure;%("", {800, 700});
        k = [1 1+n_dof 1+2*n_dof];
        for j=1:n_dof
            k = j;
            for i=1:n_slack
                subplot(n_slack,n_dof, k); hold on;
                sl_data = slack_data{i}(j,:);
                sl_data = [sl_data nan(1,length(Time) - length(sl_data))];
                plot(Time, sl_data, 'LineWidth',2.0, 'LineStyle','-', 'color', 'magenta');
                % plot(t_lim, {slack_lim[i],slack_limits[i]}, 'LineWidth',2.0, 'LineStyle','--', 'color', 'red');
                % plot(t_lim, {-slack_lim[i],-slack_limits[i]}, 'LineWidth',2.0, 'LineStyle','--', 'color', 'red');
                if (j==1), ylabel(slack_labels{i}, 'fontsize',14); end
                if (i==n_slack), xlabel('time [s]', 'fontsize',14); end
                if (i==1 && j==round(n_dof/2)), title('relaxation variables', 'fontsize',16); end
                k = k + n_dof;
            end

        end
    end
    
end