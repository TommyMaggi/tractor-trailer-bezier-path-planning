function animate_vehicles(x_ref, y_ref, truck, trailer)
    figure('Name','Truck-Trailer Animation', 'Color','white');
    hold on; axis equal; grid on;
    plot(x_ref, y_ref, 'k--', 'LineWidth', 1.5);
    
    % Vehicle parameters
    truck_length = 1.2; truck_width = 0.5;
    trailer_length = 1.0; trailer_width = 0.4;
    
    % Initialize plots
    truck_plot = patch('Vertices', zeros(4,2), 'Faces', 1:4,...
                      'FaceColor', 'none', 'EdgeColor', 'b', 'LineWidth', 2);
    trailer_plot = patch('Vertices', zeros(4,2), 'Faces', 1:4,...
                       'FaceColor', 'none', 'EdgeColor', 'r', 'LineWidth', 2);
    hitch_plot = plot([0 0], [0 0], 'g-', 'LineWidth', 2);
    
    for i = 1:20:length(truck.x)
        % Update truck
        truck_rect = get_rectangle(truck.x(i), truck.y(i),...
                                  truck.theta(i), truck_length, truck_width);
        set(truck_plot, 'Vertices', truck_rect);
        
        % Update trailer
        trailer_rect = get_rectangle(trailer.x(i), trailer.y(i),...
                                    trailer.theta(i), trailer_length, trailer_width);
        set(trailer_plot, 'Vertices', trailer_rect);
        
        % Update hitch
        hitch_x = truck.x(i) - (truck_length/2)*cos(truck.theta(i));
        hitch_y = truck.y(i) - (truck_length/2)*sin(truck.theta(i));
        set(hitch_plot, 'XData', [hitch_x trailer.x(i)+(trailer_length/2)*cos(trailer.theta(i))],...
                       'YData', [hitch_y trailer.y(i)+(trailer_length/2)*sin(trailer.theta(i+1))]);
        
        drawnow limitrate
        pause(0.15);
    end
end

function rect = get_rectangle(x, y, theta, L, W)
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    corners = [-L/2 -W/2; -L/2 W/2; L/2 W/2; L/2 -W/2];
    rect = (R * corners')' + [x y; x y; x y; x y];
end