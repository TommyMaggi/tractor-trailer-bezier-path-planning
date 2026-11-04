function animate_vehicles_with_map(map, x_ref, y_ref, truck, trailer, target_xs, target_ys)
    % This check is just in case this function is called directly again.
    if ~ishghandle(gca) % If no figure/axes exist, create them.
        figure('Name','Truck-Trailer on Open Field', 'Color','white');
        show(map); hold on; axis equal; grid on;
    else
        % Assumes we are plotting on an existing subplot/axes passed to this function.
        show(map); hold on; axis equal; grid on;
    end

    plot(x_ref, y_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Path');

    truck_length = 2.5; truck_width = 1.4;
    trailer_length = 1.5; trailer_width = 1.45;

    truck_patch = patch('Vertices', zeros(4,2), 'Faces', 1:4, 'FaceColor', 'blue', 'FaceAlpha', 0.8, 'EdgeColor', 'k');
    trailer_patch = patch('Vertices', zeros(4,2), 'Faces', 1:4, 'FaceColor', 'red', 'FaceAlpha', 0.8, 'EdgeColor', 'k');
    hitch_line = plot([0 0], [0 0], 'k-', 'LineWidth', 2);
    
    target_marker = [];
    if ~isempty(target_xs)
        target_marker = plot(NaN, NaN, 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
    end
    
    for i = 1:10:length(truck.x)
        if ~isempty(target_marker) && i <= length(target_xs)
             set(target_marker, 'XData', target_xs(i), 'YData', target_ys(i));
        end
        
        truck_rect = get_rectangle(truck.x(i), truck.y(i), truck.theta(i), truck_length, truck_width);
        set(truck_patch, 'Vertices', truck_rect);

        trailer_rect = get_rectangle(trailer.x(i), trailer.y(i), trailer.theta(i), trailer_length, trailer_width);
        set(trailer_patch, 'Vertices', trailer_rect);

        hitch_x = truck.x(i) - (truck_length/2)*cos(truck.theta(i));
        hitch_y = truck.y(i) - (truck_length/2)*sin(truck.theta(i));
        
        trailer_hitch_x = trailer.x(i) + (trailer_length/2)*cos(trailer.theta(i));
        trailer_hitch_y = trailer.y(i) + (trailer_length/2)*sin(trailer.theta(i));
        
        set(hitch_line, 'XData', [hitch_x trailer_hitch_x], 'YData', [hitch_y trailer_hitch_y]);
        
        title(sprintf('Animation Time: %.2f s', (i-1)*0.05));
        drawnow;
    end
end

function rect = get_rectangle(x, y, theta, L, W)
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    corners = [-L/2 -W/2; L/2 -W/2; L/2 W/2; -L/2 W/2];
    rect = (R * corners')' + [x y];
end