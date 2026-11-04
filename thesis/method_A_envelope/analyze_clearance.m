function [truck_clearance, trailer_clearance] = analyze_clearance(truck_sim, trailer_sim, map, geom)
    % This function calculates the minimum clearance from the tractor and trailer
    % to the nearest obstacle for every time step in a simulation run.
    %
    % INPUTS:
    %   truck_sim, trailer_sim: The simulation data structs for the truck and trailer.
    %   map: The binaryOccupancyMap of the environment.
    %   geom: The geometry struct from the geom() function.
    %
    % OUTPUTS:
    %   truck_clearance: A vector of the truck's minimum clearance at each time step.
    %   trailer_clearance: A vector of the trailer's minimum clearance at each time step.

    fprintf('--- Analyzing Vehicle Clearance ---\n');

    %% --- Step 1: Preparation ---
    fprintf('  1. Creating distance transform map for fast lookups...\n');
    distMap = bwdist(getOccupancy(map));
    
    num_sim_steps = length(truck_sim.x);
    truck_clearance = zeros(num_sim_steps, 1);
    trailer_clearance = zeros(num_sim_steps, 1);

    % --- Define local perimeter points for both vehicles ---
    % By sampling more than just the corners, we get a more accurate clearance.
    num_pts_per_side = 5; % Points along length
    num_pts_per_end = 3; % Points along width

    % Tractor perimeter (origin at its geometric center)
    L_tr = geom.Ltr; W_tr = geom.Wtr;
    local_truck_perimeter = generate_perimeter_points(L_tr, W_tr, num_pts_per_side, num_pts_per_end);

    % Trailer perimeter (origin at its geometric center)
    L_tl = geom.Ltl; W_tl = geom.Wtl;
    local_trailer_perimeter = generate_perimeter_points(L_tl, W_tl, num_pts_per_side, num_pts_per_end);

    fprintf('  2. Calculating clearance for %d time steps...\n', num_sim_steps);
    %% --- Step 2: Main Calculation Loop ---
    for i = 1:num_sim_steps
        % --- Calculate for Tractor ---
        min_dist_truck = get_min_distance_for_pose( ...
            [truck_sim.x(i), truck_sim.y(i), truck_sim.theta(i)], ...
            local_truck_perimeter, distMap, map);
        truck_clearance(i) = min_dist_truck;

        % --- Calculate for Trailer ---
        min_dist_trailer = get_min_distance_for_pose( ...
            [trailer_sim.x(i), trailer_sim.y(i), trailer_sim.theta(i)], ...
            local_trailer_perimeter, distMap, map);
        trailer_clearance(i) = min_dist_trailer;
    end
    
    fprintf('--- Clearance analysis complete ---\n');
end

% --- Helper function to generate local perimeter points ---
function local_points = generate_perimeter_points(L, W, num_side, num_end)
    front = [repmat(L/2, num_end, 1), linspace(-W/2, W/2, num_end)'];
    back  = [repmat(-L/2, num_end, 1), linspace(-W/2, W/2, num_end)'];
    left  = [linspace(-L/2, L/2, num_side)', repmat(W/2, num_side, 1)];
    right = [linspace(-L/2, L/2, num_side)', repmat(-W/2, num_side, 1)];
    local_points = unique([front; back; left; right], 'rows');
end

% --- Helper function to get the minimum distance for a single vehicle pose ---
function min_dist = get_min_distance_for_pose(pose, local_perimeter, distMap, map)
    x = pose(1);
    y = pose(2);
    theta = pose(3);
    
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    world_perimeter_points = (R * local_perimeter')' + [x, y];
    grid_coords = world2grid(map, world_perimeter_points);
    
    min_dist = inf;
    for j = 1:size(grid_coords, 1)
        r = max(1, min(grid_coords(j, 1), map.GridSize(1)));
        c = max(1, min(grid_coords(j, 2), map.GridSize(2)));
        
        dist_meters = distMap(r, c) / map.Resolution;
        if dist_meters < min_dist
            min_dist = dist_meters;
        end
    end
end
