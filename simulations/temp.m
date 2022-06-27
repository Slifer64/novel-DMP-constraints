clc;
close all;
clear;


% load('data/obs_avoid_data', 'sd_data', 'Pd_data')
load('data/obs_avoid_data_3D.mat', 'sd_data', 'Pd_data');

n_dof = size(Pd_data, 1);

X = Pd_data(1,:);
Y = Pd_data(2,:);
Z = Pd_data(3,:);
% Z = get5thOrderTrajectory(0, 0.5, 0, 1, sd_data(2)-sd_data(1));

% Pd_data = [X; Y; Z];
% save('data/obs_avoid_data_3D.mat', 'sd_data', 'Pd_data');

figure;
plot3(X, Y, Z, 'LineWidth',3);


function [y, y_dot, y_ddot] = get5thOrderTrajectory(y0, yf, t0, tf, dt)
    y0 = y0(:); % to make sure that it's a column vector
    yf = yf(:);
    Time = t0:dt:tf;
   
    T = Time(end);
    t = Time/T;
   
    y = y0 + (yf - y0) * (10*t.^3 - 15*t.^4 + 6*t.^5 );
    y_dot = (yf - y0) * (30*t.^2 - 60*t.^3 + 30*t.^4 ) / T;
    y_ddot = (yf - y0) * (60*t - 180*t.^2 + 120*t.^3 ) / T^2;
end