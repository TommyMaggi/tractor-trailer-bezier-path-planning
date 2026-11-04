% function [x_ref, y_ref] = generate_path()
%     % Closed rectangular path parameters
%     width = 20; height = 12; n_points_per_side = 500;
%     corners = [0, -height/2; width/2, -height/2;
%               width/2, height/2; -width/2, height/2;
%               -width/2, -height/2; 0, -height/2];
% 
%     x_ref = []; y_ref = [];
%     for i = 1:5
%         x_segment = linspace(corners(i,1), corners(i+1,1), n_points_per_side);
%         y_segment = linspace(corners(i,2), corners(i+1,2), n_points_per_side);
%         x_ref = [x_ref, x_segment(1:end-1)];
%         y_ref = [y_ref, y_segment(1:end-1)];
%     end
%     x_ref(end+1) = 0; y_ref(end+1) = -height/2;
% end
function [x_ref, y_ref] = generate_path()
    % Non-intersecting flower path parameters
    n_points = 2000;       % Total path points
    base_radius = 8;       % Base circle radius
    petal_amplitude = 3;   % Flower petal strength
    ripple_amplitude = 1.5;% Secondary ripple effect
    
    theta = linspace(0, 2*pi, n_points);
    
    % Complex flower equation (8 smooth petals)
    r = base_radius + petal_amplitude * cos(4*theta) + ...
        ripple_amplitude * sin(6*theta);
    
    % Convert to Cartesian coordinates
    x_ref = r .* cos(theta);
    y_ref = r .* sin(theta);
    
    % Ensure closed loop and smooth transitions
    x_ref = smoothdata([x_ref, x_ref(1)], 'gaussian', 100);
    y_ref = smoothdata([y_ref, y_ref(1)], 'gaussian', 100);
    
    % Remove duplicate end point
    x_ref = x_ref(1:end-1);
    y_ref = y_ref(1:end-1);
end