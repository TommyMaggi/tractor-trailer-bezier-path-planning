function hotspots = collect_global_hotspots_vehiclecostmap(truck_sim, trailer_sim, x_ref_path, y_ref_path, map_env, g_geom, close_call_threshold)

    fprintf(' Collecting global collision hotspots (pose adjusted for rear axle ref)...\n');
    
    hotspots = struct('vehicle_type', {}, 'vehicle_pose', {}, ...
                      'global_path_idx', {}, 'simulation_step_idx', {}, ...
                      'is_hard_collision', {}, 'is_close_call', {});

    if isempty(truck_sim.x) || isempty(trailer_sim.x)
        fprintf(' Warning: Empty simulation data.\n');
        return;
    end

    % Define Vehicle Dimensions (relative to REAR AXLE as reference point)
    % These parameters are assumed to be correctly defined for vehicleDimensions' rear-axle reference
    % Truck
    RO_truck = g_geom.Ltr * 0.15; % % RearOverhang from rear axle
    WB_truck = g_geom.Ltr * 0.70; % Wheelbase
    FO_truck = g_geom.Ltr - WB_truck - RO_truck; % FrontOverhang from front axle
    if FO_truck < -1e-6
        warning('Truck FrontOverhang calculation resulted in negative: %.3f. Clamping to 0.', FO_truck);
        FO_truck = max(0, FO_truck);
    end

    truck_dims_obj = vehicleDimensions(g_geom.Ltr, g_geom.Wtr, ...
                                     g_geom.Wtr, ... % Using Wtr for Height as placeholder
                                     'Wheelbase', WB_truck, ...
                                     'RearOverhang', RO_truck, 'FrontOverhang', FO_truck);
    
    % Trailer
    RO_trailer = g_geom.Ltl * 0.15; % % RearOverhang from rear axle
    WB_trailer = g_geom.Ltl * 0.70; % Wheelbase
    FO_trailer = g_geom.Ltl - WB_trailer - RO_trailer; % FrontOverhang from front axle
    if FO_trailer < -1e-6
        warning('Trailer FrontOverhang calculation resulted in negative: %.3f. Clamping to 0.', FO_trailer);
        FO_trailer = max(0, FO_trailer);
    end

    trailer_dims_obj = vehicleDimensions(g_geom.Ltl, g_geom.Wtl, ...
                                       g_geom.Wtl, ... % Using Wtl for Height as placeholder
                                       'Wheelbase', WB_trailer, ...
                                       'RearOverhang', RO_trailer, 'FrontOverhang', FO_trailer);

    % Collision Checkers and Costmaps
    truck_checker_hard = inflationCollisionChecker(truck_dims_obj);
    trailer_checker_hard = inflationCollisionChecker(trailer_dims_obj);

    costmap_truck_h = vehicleCostmap(map_env, 'CollisionChecker', truck_checker_hard);
    costmap_trailer_h = vehicleCostmap(map_env, 'CollisionChecker', trailer_checker_hard);

    costmap_truck_c = [];
    costmap_trailer_c = [];

    if nargin > 6 && ~isempty(close_call_threshold) && close_call_threshold > 0
        fprintf(' Setting up costmap for close call detection (threshold: %.2fm).\n', close_call_threshold);
        inflated_map = copy(map_env);
        inflate(inflated_map, close_call_threshold);
        costmap_truck_c = vehicleCostmap(inflated_map, 'CollisionChecker', truck_checker_hard);
        costmap_trailer_c = vehicleCostmap(inflated_map, 'CollisionChecker', trailer_checker_hard);
    else
        fprintf(' Close call detection is OFF for global hotspot collection.\n');
    end

    num_sim_steps = length(truck_sim.x);

    if isempty(x_ref_path) || length(x_ref_path) < 2
        fprintf(' Error: Reference path for hotspot collection is too short or empty.\n');
        return;
    end

    hard_collision_count_log = 0;
    close_call_count_log = 0; % For logging summary

    for i = 1:num_sim_steps
        sim_truck_theta_rad = truck_sim.theta(i);
        sim_trailer_theta_rad = trailer_sim.theta(i);
        
        % Convert geometric center pose (from simulation) to rear-axle pose (for checkOccupied)
        [truck_x_ra, truck_y_ra, truck_theta_deg] = convertToRearAxlePose(truck_sim.x(i), truck_sim.y(i), sim_truck_theta_rad, g_geom.Ltr, RO_truck);
        truck_pose_for_check = [truck_x_ra, truck_y_ra, truck_theta_deg];

        [trailer_x_ra, trailer_y_ra, trailer_theta_deg] = convertToRearAxlePose(trailer_sim.x(i), trailer_sim.y(i), sim_trailer_theta_rad, g_geom.Ltl, RO_trailer);
        trailer_pose_for_check = [trailer_x_ra, trailer_y_ra, trailer_theta_deg];

        is_truck_hard = checkOccupied(costmap_truck_h, truck_pose_for_check);
        is_trailer_hard = checkOccupied(costmap_trailer_h, trailer_pose_for_check);

        is_truck_close = false;
        is_trailer_close = false;
        if ~isempty(costmap_truck_c) % Only check close calls if costmap_close is configured
            if ~is_truck_hard % Only check for close call if not already a hard collision
                is_truck_close = checkOccupied(costmap_truck_c, truck_pose_for_check);
            end
            if ~is_trailer_hard % Only check for close call if not already a hard collision
                is_trailer_close = checkOccupied(costmap_trailer_c, trailer_pose_for_check);
            end
        end

        % Find closest reference path index to the TRUCK'S GEOMETRIC center for simplicity
        [~, closest_ref_idx] = min(hypot(x_ref_path - truck_sim.x(i), y_ref_path - truck_sim.y(i)));
        entry_base = struct('global_path_idx', closest_ref_idx, 'simulation_step_idx', i);
        
        if is_truck_hard
            hard_collision_count_log = hard_collision_count_log + 1;
            entry = entry_base;
            entry.vehicle_type = 'truck';
            entry.vehicle_pose = [truck_sim.x(i), truck_sim.y(i), sim_truck_theta_rad]; % Storing geometric pose
            entry.is_hard_collision = true;
            entry.is_close_call = false; % A hard collision is not also a distinct close call
            hotspots(end+1) = entry;
        elseif is_truck_close % Only add as close call if not a hard collision
            close_call_count_log = close_call_count_log + 1;
            entry = entry_base;
            entry.vehicle_type = 'truck';
            entry.vehicle_pose = [truck_sim.x(i), truck_sim.y(i), sim_truck_theta_rad]; % Storing geometric pose
            entry.is_hard_collision = false;
            entry.is_close_call = true;
            hotspots(end+1) = entry;
        end

        if is_trailer_hard
            hard_collision_count_log = hard_collision_count_log + 1;
            entry = entry_base;
            entry.vehicle_type = 'trailer';
            entry.vehicle_pose = [trailer_sim.x(i), trailer_sim.y(i), sim_trailer_theta_rad]; % Storing geometric pose
            entry.is_hard_collision = true;
            entry.is_close_call = false; % A hard collision is not also a distinct close call
            hotspots(end+1) = entry;
        elseif is_trailer_close % Only add as close call if not a hard collision
            close_call_count_log = close_call_count_log + 1;
            entry = entry_base;
            entry.vehicle_type = 'trailer';
            entry.vehicle_pose = [trailer_sim.x(i), trailer_sim.y(i), sim_trailer_theta_rad]; % Storing geometric pose
            entry.is_hard_collision = false;
            entry.is_close_call = true;
            hotspots(end+1) = entry;
        end
    end

    fprintf(' Global hotspot collection complete. Total Spots logged: %d (Hard: %d, Distinct Close: %d).\n', length(hotspots), hard_collision_count_log, close_call_count_log);
end %

% --- Local Helper Function ---
function [x_ra, y_ra, theta_deg] = convertToRearAxlePose(x_geom, y_geom, theta_rad, L_vehicle, RO_vehicle_from_rear_axle)
    % Converts a pose from geometric center (radians) to rear-axle center (degrees).
    % Inputs:
    %   x_geom, y_geom: Coordinates of the geometric center.
    %   theta_rad: Orientation in radians.
    %   L_vehicle: Total physical length of the vehicle unit.
    %   RO_vehicle_from_rear_axle: Rear Overhang, measured from the rear axle to the rearmost point.
    %                              This is the value used in vehicleDimensions 'RearOverhang'.

    % The geometric center is assumed to be at L_vehicle/2 from the front/rear.
    % Distance from the geometric center to the rear axle:
    offset_geometric_center_to_rear_axle = (L_vehicle / 2) - RO_vehicle_from_rear_axle;
    
    % Calculate rear axle position
    x_ra = x_geom - offset_geometric_center_to_rear_axle * cos(theta_rad);
    y_ra = y_geom - offset_geometric_center_to_rear_axle * sin(theta_rad);
    
    % Convert angle to degrees
    theta_deg = rad2deg(theta_rad);
end