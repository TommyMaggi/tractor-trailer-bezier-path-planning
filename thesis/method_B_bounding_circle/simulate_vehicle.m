function [truck, trailer, target_xs, target_ys, bounding_circle_data] = simulate_vehicle(x_ref, y_ref, openField, g)
    % This function now calculates the bounding circle based on the moving
    % midpoint of the entire vehicle assembly.

    % Get all parameters from the geometry struct
    v = g.v; dt = g.dt; lookahead = g.look;
    truck_length = g.Ltr; trailer_length = g.Ltl; hitch_length = g.Lh;
    bounding_radius = g.bounding_radius;

    distMap = bwdist(getOccupancy(openField));
    map_resolution = openField.Resolution;
    
    nSteps = length(x_ref);
    truck = struct('x', zeros(nSteps,1), 'y', zeros(nSteps,1), 'theta', zeros(nSteps,1));
    trailer = struct('x', zeros(nSteps,1), 'y', zeros(nSteps,1), 'theta', zeros(nSteps,1));
    target_xs = zeros(nSteps, 1); target_ys = zeros(nSteps, 1);
    bounding_circle_data = struct('center_x', zeros(nSteps,1), 'center_y', zeros(nSteps,1), ...
                                  'radius', bounding_radius, 'is_intersecting', false(nSteps,1));
    
    truck.x(1) = x_ref(1); truck.y(1) = y_ref(1);
    truck.theta(1) = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
    hitch_x_init = truck.x(1) - (truck_length/2)*cos(truck.theta(1));
    hitch_y_init = truck.y(1) - (truck_length/2)*sin(truck.theta(1));
    trailer.x(1) = hitch_x_init - (hitch_length)*cos(trailer.theta(1));
    trailer.y(1) = hitch_y_init - (hitch_length)*sin(trailer.theta(1));
    
    for i = 1:nSteps-1
        % Pure Pursuit (unchanged)
        truck_front_x_pp = truck.x(i) + (truck_length/2) * cos(truck.theta(i));
        truck_front_y_pp = truck.y(i) + (truck_length/2) * sin(truck.theta(i));
        [~, idx] = min((x_ref - truck_front_x_pp).^2 + (y_ref - truck_front_y_pp).^2);
        local_lookahead = lookahead;
        if idx > nSteps/2, local_lookahead = lookahead / 2; end
        lookahead_idx = min(idx + max(1, floor(local_lookahead/(v*dt))), nSteps);
        target_x = x_ref(lookahead_idx); target_y = y_ref(lookahead_idx);
        target_xs(i) = target_x; target_ys(i) = target_y;
        alpha = atan2(target_y - truck.y(i), target_x - truck.x(i)) - truck.theta(i);
        delta = atan2(2*truck_length*sin(alpha), lookahead);
        
        % Update vehicle states (unchanged)
        truck.x(i+1) = truck.x(i) + v*cos(truck.theta(i))*dt;
        truck.y(i+1) = truck.y(i) + v*sin(truck.theta(i))*dt;
        truck.theta(i+1) = truck.theta(i) + v*tan(delta)/truck_length*dt;
        hitch_x = truck.x(i+1) - (truck_length/2)*cos(truck.theta(i+1));
        hitch_y = truck.y(i+1) - (truck_length/2)*sin(truck.theta(i+1));
        trailer.theta(i+1) = atan2(hitch_y - trailer.y(i), hitch_x - trailer.x(i));
        trailer.x(i+1) = hitch_x - (hitch_length+trailer_length/2)*cos(trailer.theta(i+1));
        trailer.y(i+1) = hitch_y - (hitch_length+trailer_length/2)*sin(trailer.theta(i+1));

        % --- CORRECTED Bounding Circle Calculation ---
        % 1. Find the two extreme points of the vehicle assembly
        truck_front_x = truck.x(i+1) + (truck_length/2) * cos(truck.theta(i+1));
        truck_front_y = truck.y(i+1) + (truck_length/2) * sin(truck.theta(i+1));
        trailer_rear_x = trailer.x(i+1) - (trailer_length/2) * cos(trailer.theta(i+1));
        trailer_rear_y = trailer.y(i+1) - (trailer_length/2) * sin(trailer.theta(i+1));
        
        % 2. The circle's center is the midpoint between these two extremes
        circle_center_x = (truck_front_x + trailer_rear_x) / 2;
        circle_center_y = (truck_front_y + trailer_rear_y) / 2;
        
        bounding_circle_data.center_x(i+1) = circle_center_x;
        bounding_circle_data.center_y(i+1) = circle_center_y;
        
        % 3. Perform proximity check using the new center
        grid_coords = world2grid(openField, [circle_center_x, circle_center_y]);
        dist_to_obstacle = distMap(grid_coords(1), grid_coords(2)) * (1/map_resolution);
        bounding_circle_data.is_intersecting(i+1) = dist_to_obstacle < bounding_radius;
    end
    
    truck.x(end) = x_ref(end); truck.y(end) = y_ref(end);
    truck.theta(end) = atan2(diff(y_ref(end-1:end)), diff(x_ref(end-1:end)));
end