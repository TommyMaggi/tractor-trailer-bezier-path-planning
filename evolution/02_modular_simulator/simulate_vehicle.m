function [truck, trailer] = simulate_vehicle(x_ref, y_ref)
    % Parameters
    truck_length = 1.2; trailer_length = 1.0;
    hitch_length = 0.6; v = 0.5; dt = 0.1; lookahead = 1.5;
    
    % Initialization
    nSteps = length(x_ref);
    truck = struct('x', zeros(nSteps,1), 'y', zeros(nSteps,1), 'theta', zeros(nSteps,1));
    trailer = struct('x', zeros(nSteps,1), 'y', zeros(nSteps,1), 'theta', zeros(nSteps,1));
    
    % Initial conditions
    truck.x(1) = x_ref(1); truck.y(1) = y_ref(1);
    truck.theta(1) = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
    
    % Trailer initial position (front connected to hitch)
    hitch_x = truck.x(1) - (truck_length/2)*cos(truck.theta(1));
    hitch_y = truck.y(1) - (truck_length/2)*sin(truck.theta(1));
    trailer.x(1) = hitch_x - (hitch_length)*cos(trailer.theta(1));
    trailer.y(1) = hitch_y - (hitch_length)*sin(trailer.theta(1));

    for i = 1:nSteps-1
        % Pure Pursuit for truck
        [~, idx] = min(sqrt((x_ref - truck.x(i)).^2 + (y_ref - truck.y(i)).^2));
        lookahead_idx = min(idx + floor(lookahead/(v*dt)), nSteps);
        target_x = x_ref(lookahead_idx); target_y = y_ref(lookahead_idx);
        
        alpha = atan2(target_y - truck.y(i), target_x - truck.x(i)) - truck.theta(i);
        delta = atan2(2*truck_length*sin(alpha), lookahead);
        
        truck.theta(i+1) = truck.theta(i) + v*tan(delta)/truck_length*dt;
        truck.x(i+1) = truck.x(i) + v*cos(truck.theta(i+1))*dt;
        truck.y(i+1) = truck.y(i) + v*sin(truck.theta(i+1))*dt;
        
        % Calculate hitch point (truck rear center)
        hitch_x = truck.x(i+1) - (truck_length/2)*cos(truck.theta(i+1));
        hitch_y = truck.y(i+1) - (truck_length/2)*sin(truck.theta(i+1));
        
        % Update trailer (come la simulazione p5js)
        trailer.theta(i+1) = atan2(hitch_y - trailer.y(i), hitch_x - trailer.x(i));
        trailer.x(i+1) = hitch_x - (hitch_length+trailer_length/2)*cos(trailer.theta(i+1));
        trailer.y(i+1) = hitch_y - (hitch_length+trailer_length/2)*sin(trailer.theta(i+1));
    end
end