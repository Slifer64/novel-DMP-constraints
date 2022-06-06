clc;
close all;
clear;

% rng(0);


load('data/obs_avoid_data')


ellipsoid{1} = struct('Sigma',getEllipseSigma(45, 0.15, 0.08), 'c',[-0.2; 0.3]);
ellipsoid{2} = struct('Sigma',getEllipseSigma(-45, 0.1, 0.06), 'c',[0.16; 0.57]);
ellipsoid_colors = {[1.0, 0.4, 0], [0.4, 0.2, 0]};

for i=1:length(ellipsoid)  
    E_p{i} = drawElipsoid2D(ellipsoid{i}.Sigma, ellipsoid{i}.c);
end

figure; hold on;
plot(Pd_data(1, :), Pd_data(2, :), 'LineWidth',2);
for i=1:length(E_p)
    plot(E_p{i}(1,:), E_p{i}(2,:), 'LineWidth',2, 'Color',ellipsoid_colors{i});
    plot(ellipsoid{i}.c(1), ellipsoid{i}.c(2), 'LineWidth',2, 'Marker','x', 'LineStyle','None', 'Markersize',14, 'Color',ellipsoid_colors{i});
end
axis equal


%% Define Ellipsoid

% theta = 45 * pi / 180;
% R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
% Lambda_2 = diag([1.2, 0.5]);
% Sigma = R * Lambda_2.^2 * R';
% c = [0.5; 0.2];

theta = rand() * pi / 180;
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
Lambda_2 = diag([2*rand(), 2*rand()]);
Sigma = R * Lambda_2.^2 * R';
c = [rand(); rand()];

%% Define a point inside the ellipsoid
r = 0.2 + 0.6*rand();
theta = rand()*2*pi;
p1 = c + chol(Sigma, 'lower')*r*[cos(theta); sin(theta)];

%% Find the point on the surface ellipsoid
lambda = 1 / sqrt((p1-c)'/Sigma*(p1-c));
p2 = c + lambda*(p1-c);

%% Find the norm to the ellipsoid on that point
n = Sigma \ (p2 - c);
n = n / norm(n);

%% calculate two points on tangent line
v = [n(2); - n(1)]; % tangent line direction
p3 = p2 - 0.4*v;
p4 = p2 + 0.4*v;

%% plot
p = drawElipsoid2D(Sigma, c);

figure; hold on;
plot(p(1,:), p(2,:), 'LineWidth',2, 'Color','blue')
quiver(p2(1), p2(2), n(1), n(2), 0.5, 'Color','green', 'LineWidth',3);
plot([p3(1) p4(1)], [p3(2) p4(2)], 'LineWidth',2.2, 'LineStyle','-.', 'Color',[0 0.8 0]);
plot([c(1) p2(1)], [c(2) p2(2)], 'LineWidth',2, 'LineStyle',':', 'Color','cyan');
plot(c(1), c(2), 'LineWidth',2, 'Marker','x', 'LineStyle','None', 'Markersize',14, 'Color','blue');
plot(p1(1), p1(2), 'LineWidth',2, 'Marker','x', 'LineStyle','None', 'Markersize',14, 'Color','red');
plot(p2(1), p2(2), 'LineWidth',2, 'Marker','x', 'LineStyle','None', 'Markersize',14, 'Color','magenta');
axis equal


%% ===================================
%% ===================================

function p = drawElipsoid2D(Sigma, c)

    theta = linspace(0, 2*pi, 200);
    L = chol(Sigma, 'lower'); % sqrtm(Sigma);
    p = c + L*[cos(theta); sin(theta)];

end

function Sigma = getEllipseSigma(angle, lambda_x, lambda_y)

    theta = angle * pi / 180;
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    Lambda_2 = diag([lambda_x, lambda_y]);
    Sigma = R * Lambda_2.^2 * R';

end
