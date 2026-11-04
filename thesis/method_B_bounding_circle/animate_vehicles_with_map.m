function animate_vehicles_with_map(map, x_ref, y_ref, truck, trailer, target_xs, target_ys, bounding_circle_data, is_dangerous_flags)
    % This is the corrected function that animates the simulation and
    % visualizes the bounding circle.

    figure('Name','Truck-Trailer Simulation with Bounding Circle', 'Color','white');
    show(map); hold on; axis equal; grid on;
    
    % Plot the reference trajectory
    plot(x_ref, y_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Path');
    
    % Vehicle dimensions for drawing patches
    truck_length = 2.5; truck_width = 1.4;
    trailer_length = 1.5; trailer_width = 1.45;
    
    % --- Initialize all graphics objects ONCE before the loop ---
    truck_patch = patch('Vertices', zeros(4,2), 'Faces', 1:4, 'FaceColor', 'none', 'EdgeColor', 'b', 'LineWidth', 2);
    trailer_patch = patch('Vertices', zeros(4,2), 'Faces', 1:4, 'FaceColor', 'none', 'EdgeColor', 'r', 'LineWidth', 2);
    hitch_line = plot([0 0], [0 0], 'g-', 'LineWidth', 2);
    target_marker = plot(NaN, NaN, 'go', 'MarkerSize', 5);
    
    % Initialize Bounding Circle Plot
    radius = bounding_circle_data.radius;
    theta_circ = linspace(0, 2*pi, 50);
    circle_x_unit = cos(theta_circ);
    circle_y_unit = sin(theta_circ);
    bounding_circle_plot = plot(NaN, NaN, 'y--', 'LineWidth', 1.0);
    
    % --- Main animation loop ---
    for i = 1:20:length(truck.x)
        % Update target marker position
        set(target_marker, 'XData', target_xs(i), 'YData', target_ys(i));
        
        % Update truck patch
        truck_rect = get_rectangle(truck.x(i), truck.y(i), truck.theta(i), truck_length, truck_width);
        set(truck_patch, 'Vertices', truck_rect);
        
        % Update trailer patch
        trailer_rect = get_rectangle(trailer.x(i), trailer.y(i), trailer.theta(i), trailer_length, trailer_width);
        set(trailer_patch, 'Vertices', trailer_rect);
        
        % Update hitch connection line
        hitch_x = truck.x(i) - (truck_length/2)*cos(truck.theta(i));
        hitch_y = truck.y(i) - (truck_length/2)*sin(truck.theta(i));
        set(hitch_line, 'XData', [hitch_x trailer.x(i)+(trailer_length/2)*cos(trailer.theta(i))], ...
                        'YData', [hitch_y trailer.y(i)+(trailer_length/2)*sin(trailer.theta(i))]);
        
        % Update Bounding Circle Visualization
        center_x = bounding_circle_data.center_x(i);
        center_y = bounding_circle_data.center_y(i);
        is_dangerous = is_dangerous_flags(i);
        
        set(bounding_circle_plot, 'XData', circle_x_unit * radius + center_x, ...
                                  'YData', circle_y_unit * radius + center_y);
        if is_dangerous
            set(bounding_circle_plot, 'Color', 'magenta', 'LineStyle', '-'); % Solid Purple for danger
        else
            set(bounding_circle_plot, 'Color', [0.9 0.9 0.2], 'LineStyle', '--'); % Dashed Yellow for safe
        end
        
        % --- ADDED BACK: Commands to force graphics update and pause ---
        drawnow limitrate;
        pause(0.1);
    end
end

% Helper function to calculate rectangle vertices
function rect = get_rectangle(x, y, theta, L, W)
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    corners = [-L/2 -W/2; -L/2 W/2; L/2 W/2; L/2 -W/2];
    rect = (R * corners')' + [x y];
end