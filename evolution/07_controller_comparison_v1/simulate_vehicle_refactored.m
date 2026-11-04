function results = simulate_vehicle_refactored(x_ref, y_ref, path_curvatures, params)
    % Unpack parameters
    g = params.geom;
    v = params.sim.v;
    dt = params.sim.dt;
    t_end = params.sim.t_end; % Get simulation end time
    
    % Initialization
    nSteps = ceil(t_end / dt) + 1; % Ensure nSteps is long enough
    
    truck = struct('x', zeros(nSteps,1), 'y', zeros(nSteps,1), 'theta', zeros(nSteps,1));
    trailer = struct('x', zeros(nSteps,1), 'y', zeros(nSteps,1), 'theta', zeros(nSteps,1));
    
    time_hist = zeros(nSteps,1);
    steering_hist = zeros(nSteps,1);
    lookahead_hist = zeros(nSteps,1);

    % Initial conditions
    truck.x(1) = x_ref(1);
    truck.y(1) = y_ref(1);
    truck.theta(1) = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
    
    hitch_x_init = truck.x(1) - (g.truck_length/2) * cos(truck.theta(1));
    hitch_y_init = truck.y(1) - (g.truck_length/2) * sin(truck.theta(1));
    trailer.theta(1) = truck.theta(1);
    trailer.x(1) = hitch_x_init - (g.hitch_length + g.trailer_length/2) * cos(trailer.theta(1));
    trailer.y(1) = hitch_y_init - (g.hitch_length + g.trailer_length/2) * sin(trailer.theta(1));
    
    last_idx = nSteps - 1; 
    
    % Main simulation loop
    for i = 1:nSteps-1
        current_time = (i-1)*dt;
        
        % --- Time-based stop condition ---
        if current_time >= t_end
            last_idx = i - 1;
            disp('Simulation end time reached.');
            break;
        end
        
        % (The rest of the simulation logic is unchanged)
        % --- 1. Find the reference point on the truck ---
        if strcmp(params.controlPoint, 'front')
            control_x = truck.x(i) + (g.truck_length/2) * cos(truck.theta(i));
            control_y = truck.y(i) + (g.truck_length/2) * sin(truck.theta(i));
        else % 'rear'
            control_x = truck.x(i) - (g.truck_length/2) * cos(truck.theta(i));
            control_y = truck.y(i) - (g.truck_length/2) * sin(truck.theta(i));
        end

        % --- 2. Find closest point on path and determine lookahead ---
        [~, nearest_idx] = min((x_ref - control_x).^2 + (y_ref - control_y).^2);
        
        if strcmp(params.lookaheadType, 'adaptive')
            p = params.control;
            if params.sharp_turn_zones(nearest_idx)
                current_lookahead = p.minLookahead;
            else
                current_curvature = abs(path_curvatures(nearest_idx));
                lookahead = p.maxLookahead / (1 + p.curvature_gain * current_curvature);
                current_lookahead = max(p.minLookahead, min(p.maxLookahead, lookahead));
            end
        else % 'fixed'
            current_lookahead = params.control.fixedLookahead;
        end
        
        % --- 3. Find the target point (clamp to end of path) ---
        lookahead_idx = nearest_idx;
        nRefPoints = length(x_ref);
        while lookahead_idx < nRefPoints && norm([x_ref(lookahead_idx) - control_x, y_ref(lookahead_idx) - control_y]) < current_lookahead
            lookahead_idx = lookahead_idx + 1;
        end
        
        target_x = x_ref(lookahead_idx);
        target_y = y_ref(lookahead_idx);
        
        % --- 4. Calculate steering angle (delta) ---
        alpha = atan2(target_y - control_y, target_x - control_x) - truck.theta(i);
        delta = atan2(2 * g.truck_length * sin(alpha), current_lookahead);
        
        % --- 5. Update vehicle kinematics ---
        truck.x(i+1) = truck.x(i) + v * cos(truck.theta(i)) * dt;
        truck.y(i+1) = truck.y(i) + v * sin(truck.theta(i)) * dt;
        truck.theta(i+1) = truck.theta(i) + (v / g.truck_length) * tan(delta) * dt;

        hitch_x = truck.x(i+1) - (g.truck_length/2) * cos(truck.theta(i+1));
        hitch_y = truck.y(i+1) - (g.truck_length/2) * sin(truck.theta(i+1));
        
        trailer.theta(i+1) = atan2(hitch_y - trailer.y(i), hitch_x - trailer.x(i));
        trailer_center_dist = g.hitch_length + g.trailer_length/2;
        trailer.x(i+1) = hitch_x - trailer_center_dist * cos(trailer.theta(i+1));
        trailer.y(i+1) = hitch_y - trailer_center_dist * sin(trailer.theta(i+1));
        
        % --- 6. Log data ---
        time_hist(i) = current_time;
        steering_hist(i) = delta;
        lookahead_hist(i) = current_lookahead;
    end
    
    % Package results
    results.truck.x = truck.x(1:last_idx);
    results.truck.y = truck.y(1:last_idx);
    results.truck.theta = truck.theta(1:last_idx);
    
    results.trailer.x = trailer.x(1:last_idx);
    results.trailer.y = trailer.y(1:last_idx);
    results.trailer.theta = trailer.theta(1:last_idx);
    
    results.time = time_hist(1:last_idx);
    results.steering_hist = steering_hist(1:last_idx);
    results.lookahead_hist = lookahead_hist(1:last_idx);
end