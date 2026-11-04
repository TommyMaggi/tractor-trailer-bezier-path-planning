function animate_vehicles_with_map(map, x_ref, y_ref, truck, trailer, target_xs,target_ys)
    figure('Name','Truck-Trailer on Open Field', 'Color','white');
    show(map); hold on; axis equal; grid on;

    % Plot the reference trajectory
    plot(x_ref, y_ref, 'k--', 'LineWidth', 1.5);

    % Vehicle dimensions
    truck_length = 2.2; truck_width = 1;
    trailer_length = 1.5; trailer_width = 1;

    % Initialize vehicle patches
    truck_patch = patch('Vertices', zeros(4,2), 'Faces', 1:4,...
                        'FaceColor', 'none', 'EdgeColor', 'b', 'LineWidth', 2);
    trailer_patch = patch('Vertices', zeros(4,2), 'Faces', 1:4,...
                          'FaceColor', 'none', 'EdgeColor', 'r', 'LineWidth', 2);
    hitch_line = plot([0 0], [0 0], 'g-', 'LineWidth', 2);

    for i = 1:20:length(truck.x)
        target_marker = plot(NaN, NaN, 'go', 'MarkerSize', 5);
        set(target_marker, 'XData', target_xs(i), 'YData', target_ys(i));
        
        % Truck
        truck_rect = get_rectangle(truck.x(i), truck.y(i), truck.theta(i), truck_length, truck_width);
        set(truck_patch, 'Vertices', truck_rect);

        % Trailer
        trailer_rect = get_rectangle(trailer.x(i), trailer.y(i), trailer.theta(i), trailer_length, trailer_width);
        set(trailer_patch, 'Vertices', trailer_rect);

        % Hitch connection line
        hitch_x = truck.x(i) - (truck_length/2)*cos(truck.theta(i));
        hitch_y = truck.y(i) - (truck_length/2)*sin(truck.theta(i));
        set(hitch_line, 'XData', [hitch_x trailer.x(i)+(trailer_length/2)*cos(trailer.theta(i))],...
                        'YData', [hitch_y trailer.y(i)+(trailer_length/2)*sin(trailer.theta(i))]);
        
        drawnow limitrate;
        pause(0.1);
    end
  end

function rect = get_rectangle(x, y, theta, L, W)
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    corners = [-L/2 -W/2; -L/2 W/2; L/2 W/2; L/2 -W/2];
    rect = (R * corners')' + [x y; x y; x y; x y];
end

