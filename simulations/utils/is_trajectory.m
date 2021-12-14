function is_traj = is_trajectory(Time, Pd_data, dPd_data, ddPd_data)

n_data = length(Time);
dt = diff(Time);

P_data = zeros(3,n_data);
dP_data = zeros(3,n_data);
ddP_data = zeros(3,n_data);

P = Pd_data(:,1);
dP = zeros(3,1);
ddP = ddPd_data(:,1) + 30*(dPd_data(:,1) - dP) + 100*(Pd_data(:,1) - P);
for j=2:n_data
    
    % trajectory at instant j-1
    P_data(:,j-1) = P;
    dP_data(:,j-1) = dP;
    ddP_data(:,j-1) = ddP;
    
    % trajectory at instant j
    P = P + dP*dt(j-1);
    dP = dP + ddP*dt(j-1);
    ddP = ddPd_data(:,j) + 100*(dPd_data(:,j) - dP) + 500*(Pd_data(:,j) - P);
    
end

P_data(:,end) = P;
dP_data(:,end) = dP;
ddP_data(:,end) = ddP;


figure;
k = [1 4 7];
for i=1:3
    subplot(3,3,k(1)); hold on;
    plot(Time, P_data(i,:), 'LineWidth',2, 'Color','blue');
    plot(Time, Pd_data(i,:), 'LineWidth',2, 'LineStyle',':', 'Color','magenta');
    
    subplot(3,3,k(2)); hold on;
    plot(Time, dP_data(i,:), 'LineWidth',2, 'Color','blue');
    plot(Time, dPd_data(i,:), 'LineWidth',2, 'LineStyle',':', 'Color','magenta');
    
    subplot(3,3,k(3)); hold on;
    plot(Time, ddP_data(i,:), 'LineWidth',2, 'Color','blue');
    plot(Time, ddPd_data(i,:), 'LineWidth',2, 'LineStyle',':', 'Color','magenta');
    
    k = k + 1;
end

figure;
k = [1 4 7];
for i=1:3
    subplot(3,3,k(1));
    plot(Time, abs(Pd_data(i,:)-P_data(i,:)), 'LineWidth',2, 'Color','red');
    
    subplot(3,3,k(2));
    plot(Time, abs(Pd_data(i,:)-P_data(i,:)), 'LineWidth',2, 'Color','red');
    
    subplot(3,3,k(3));
    plot(Time, abs(Pd_data(i,:)-P_data(i,:)), 'LineWidth',2, 'Color','red');
    
    k = k + 1;
end

err_pos = max(abs(Pd_data(i,:)-P_data(i,:)))
err_vel = max(abs(dPd_data(i,:)-dP_data(i,:)))
err_accel = max(abs(ddPd_data(i,:)-ddP_data(i,:)))

is_traj = (err_pos < 1e-3 && err_vel < 1e-3 && err_accel < 1e-2)

end