function cte = calculate_cte(ref_x, ref_y, vehicle_x, vehicle_y)
    % Calculates the Cross-Track Error (CTE) for each point in a vehicle's trajectory.
    
    num_vehicle_points = length(vehicle_x);
    cte = zeros(num_vehicle_points, 1);
    ref_path = [ref_x(:), ref_y(:)];

    for i = 1:num_vehicle_points
        % Current vehicle position
        current_pos = [vehicle_x(i), vehicle_y(i)];
        
        % Find the closest point on the reference path
        [~, min_idx] = min(sum((ref_path - current_pos).^2, 2));
        
        % To calculate the sign of the error, we need a vector representing path direction
        if min_idx < length(ref_x)
            path_p1 = ref_path(min_idx, :);
            path_p2 = ref_path(min_idx + 1, :);
        else % We are at the last point of the path
            path_p1 = ref_path(min_idx - 1, :);
            path_p2 = ref_path(min_idx, :);
        end
        
        % Vector from the path to the vehicle
        path_to_vehicle = current_pos - path_p1;
        
        % Vector representing the path's direction
        path_direction = path_p2 - path_p1;
        
        % The sign of the z-component of the cross product tells us if we are left or right
        cross_prod_z = path_direction(1) * path_to_vehicle(2) - path_direction(2) * path_to_vehicle(1);
        
        % The distance is the magnitude of the error
        distance = sqrt(min(sum((ref_path - current_pos).^2, 2)));
        
        % Assign the signed error
        cte(i) = sign(cross_prod_z) * distance;
    end
end