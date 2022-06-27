clc;
close all;
clear;

% rng(0);


%% Define Ellipsoid
% Sigma = getEllipseSigma([30, 20, 90], [1, 2, 0.5]);
% c = [0.1; -0.8; 0.5];
Sigma = getEllipseSigma([0, 0, 45], [0.19, 0.06, 0.05]);
c = [-0.14; 0.28; 0.1];

%% Define a point inside the ellipsoid
% r = 0.2 + 0.6*rand();
% theta = rand()*2*pi;
% phi = rand()*pi;
% p1 = c + chol(Sigma, 'lower')*r*[sin(phi)*cos(theta); sin(phi)*sin(theta); cos(phi)];
p1 = [-0.2825; 0.2439; 0.1014];

%% Find the point on the surface ellipsoid
lambda = 1 / sqrt((p1-c)'/Sigma*(p1-c));
p_e = c + lambda*(p1-c);

%% Find the norm to the ellipsoid on that point
n = Sigma \ (p_e - c);
n = n / norm(n);

%% calculate tangent plane
p_e = [-0.2868; 0.2428; 0.1014];
n = [-0.2800, -0.9447, 0.1705];

[x_plane, y_plane, z_plane] = createPlane(p_e, n, 0.05);


%% plot
[X, Y, Z] = drawElipsoid(Sigma, c);

figure; hold on;
sh = surf(X, Y, Z, 'LineStyle','None', 'FaceColor',[0.85 0.33 0.1], 'FaceAlpha',0.5);
patch_h = patch(x_plane, y_plane, z_plane, 'green', 'FaceAlpha',0.2, 'LineStyle','none');
quiver3(p_e(1), p_e(2), p_e(3), n(1), n(2), n(3), 0.5, 'Color','green', 'LineWidth',3);
% plot3([p3(1) p4(1)], [p3(2) p4(2)], 'LineWidth',2.2, 'LineStyle','-.', 'Color',[0 0.8 0]);
plot3([c(1) p_e(1)], [c(2) p_e(2)], [c(3) p_e(3)], 'LineWidth',2, 'LineStyle',':', 'Color','cyan');
plot3(c(1), c(2), c(3),  'LineWidth',3, 'Marker','*', 'LineStyle','None', 'Markersize',16, 'Color','blue');
plot3(p1(1), p1(2), p1(3), 'LineWidth',3, 'Marker','x', 'LineStyle','None', 'Markersize',16, 'Color','red');
plot3(p_e(1), p_e(2), p_e(3), 'LineWidth',3, 'Marker','x', 'LineStyle','None', 'Markersize',16, 'Color','magenta');
axis equal
grid on;
view(-148.33, 5.3529);


%% ===================================
%% ===================================

function [X, Y, Z] = drawElipsoid(Sigma, c)

    theta = linspace(0, 2*pi, 20);
    phi = linspace(0, pi, 20);
    
    [Theta, Phi] = meshgrid(theta, phi);
    
    L = chol(Sigma, 'lower'); % sqrtm(Sigma);
    X = sin(Phi).*cos(Theta);
    Y = sin(Phi).*sin(Theta);
    Z = cos(Phi);
    
    P = [X(:)'; Y(:)'; Z(:)'];
    P = c + L*P;
    
    X = reshape(P(1,:), length(theta), length(phi));
    Y = reshape(P(2,:), length(theta), length(phi));
    Z = reshape(P(3,:), length(theta), length(phi));
    
end

function [X, Y, Z] = createPlane(center, normal, scale)
    
    % Ax + By + Cz + D = 0
    A = normal(1);
    B = normal(2);
    C = normal(3);
    D = -dot(normal, center);
    X = center(1) + scale*[1 -1 -1 1]; % Generate data for x vertices
    Y = center(2) + scale*[1 1 -1 -1]; % Generate data for y vertices
    Z = -1/C*(A*X + B*Y + D); % Solve for z vertices data

end


function Sigma = getEllipseSigma(euler_ang, axes_len)

    R = eul2rotm(euler_ang);
    Sigma = R * diag(axes_len).^2 * R';

end
