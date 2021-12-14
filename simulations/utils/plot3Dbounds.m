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