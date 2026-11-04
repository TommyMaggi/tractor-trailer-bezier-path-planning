%% vineyardNavigator_SegmentBySegment.m
%% ------------------------------------------------------------------------
%                           SETUP & INITIALIZATION
%  ------------------------------------------------------------------------

% --- Environment Setup ---
clear;
clc; 
close all; 

fprintf('--- Vineyard Navigation System (Segment-by-Segment with Modification) ---\n');


%% --- 1. Load Vineyard Parameters & Initialize Occupancy Map ---

VINEYARD_PARAMS = defineVineyardParameters();
g_config.map_resolution = 10; % Cells per meter

fprintf('Step 0: Generating Occupancy Map from filari.png...\n');

% Load map image
try
    topView = imread('filari.png');
catch ME
    error('filari.png not found in the current directory. %s', ME.message);
end

% Convert to binary occupancy grid
if size(topView, 3) == 3
    topView = rgb2gray(topView);
end
imageMask = imbinarize(topView);

% Invert mask if necessary. Obstacles should be '1'.
% IMPORTANT: Assumes filari.png has BLACK obstacles on a WHITE background.
% If your map has WHITE obstacles on a BLACK background, COMMENT OUT THIS LINE.
imageMask = ~imageMask;

% Create the binaryOccupancyMap object
imageMaskResized = imresize(imageMask, [50 * g_config.map_resolution, 50 * g_config.map_resolution]);
environmentMap = binaryOccupancyMap(50, 50, g_config.map_resolution);
setOccupancy(environmentMap, imageMaskResized);

% Update vineyard parameters with map-specific offsets
UPDATED_VINEYARD_PARAMS = VINEYARD_PARAMS;
UPDATED_VINEYARD_PARAMS.map_origin_offset = [environmentMap.XWorldLimits(1), environmentMap.YWorldLimits(1)];
UPDATED_VINEYARD_PARAMS.row_y_start_param_ref = VINEYARD_PARAMS.row_y_start;
UPDATED_VINEYARD_PARAMS.row_y_end_param_ref = VINEYARD_PARAMS.row_y_end;

% --- Visualization ---
fig_main_handle = figure('Name', 'Vineyard Task Environment', 'Position', [50 50 900 750]);
ax_main = axes(fig_main_handle);
show(environmentMap, 'Parent', ax_main);
hold(ax_main, 'on');
title(ax_main, 'Vineyard Map & Task Waypoints');
axis(ax_main, 'equal');
grid(ax_main, 'on');


%% --- 1.5 Initialize Probabilistic Roadmap (PRM) ---

fprintf('Initializing PRM...\n');

% Create a slightly inflated map for robust PRM planning
planningMap = copy(environmentMap);
prmInflateRadius = 0.75; % meters
inflate(planningMap, prmInflateRadius);

% Build the PRM roadmap (nodes and connections)
[prm_nodes_initial, prm_adjacency_initial] = buildPRMRoadmap(planningMap, environmentMap, 500, 20);

if isempty(prm_nodes_initial)
    error('PRM Initialization Failed. No nodes were generated.');
end

fprintf('PRM: %d nodes and %d edges built.\n', size(prm_nodes_initial, 1), nnz(prm_adjacency_initial) / 2);

% Plot PRM nodes
plot(ax_main, prm_nodes_initial(:,1), prm_nodes_initial(:,2), 'c.', ...
    'MarkerSize', 6, 'DisplayName', 'PRM Nodes');


%% --- 2. Define Snap Points, Custom Points & Task Sequence ---

% Collect all predefined entry/exit points from the vineyard parameters
all_snap_points = collectAllSnapPoints(UPDATED_VINEYARD_PARAMS);

        % Define a list of custom, user-defined waypoints
custom_points_list = { ...
    {'name', 'custom_A', 'pose', [43, 25, deg2rad(90)]}, ...
    {'name', 'custom_B', 'pose', [21, 28, deg2rad(-90)]}, ...
    {'name', 'custom_C', 'pose', [30, 11, deg2rad(90)]} ...
};

% Convert the cell array of custom points to a more accessible struct array
all_custom_points_struct = struct('name', {}, 'xy', {}, 'theta_rad', {});
for k = 1:length(custom_points_list)
    all_custom_points_struct(k).name = custom_points_list{k}{2};
    all_custom_points_struct(k).xy = custom_points_list{k}{4}(1:2);
    all_custom_points_struct(k).theta_rad = custom_points_list{k}{4}(3);
end

% Define the full sequence of tasks for the robot to execute
task_sequence_with_custom = { ...
    {'type', 'snap',   'name', 'Row_1_BottomEntry'}, ...
    {'type', 'snap',   'name', 'Row_1_TopExit'}, ...
    {'type', 'snap',   'name', 'Row_2_TopEntry'}, ...
    {'type', 'snap',   'name', 'Row_2_BottomExit'}, ...
    {'type', 'snap',   'name', 'Row_3_BottomEntry'}, ...
    {'type', 'snap',   'name', 'Row_3_TopExit'}, ...
    {'type', 'snap',   'name', 'Row_1_TopEntry'}, ...
    {'type', 'snap',   'name', 'Row_1_BottomExit'}, ...
    {'type', 'custom', 'name', 'custom_A'}, ...
    {'type', 'custom', 'name', 'custom_B'}  ...
    {'type', 'custom', 'name', 'custom_C'}  ...
};

% Convert the task sequence into a structured array of waypoints
task_waypoints_full_sequence = getWaypointsFromMixedTask(...
    task_sequence_with_custom, all_snap_points, all_custom_points_struct);

% Plot the task waypoints on the main figure
for i = 1:length(task_waypoints_full_sequence)
    if strcmp(task_waypoints_full_sequence(i).waypoint_type, 'custom')
        plot_color = 'cd';
        marker_face_color = 'c';
    else
        plot_color = 'yd';
        marker_face_color = 'y';
    end

    plot(ax_main, task_waypoints_full_sequence(i).xy(1), task_waypoints_full_sequence(i).xy(2), ...
        plot_color, 'MarkerFaceColor', marker_face_color, 'MarkerSize', 7, 'DisplayName', 'Task Waypoint');

    text(ax_main, task_waypoints_full_sequence(i).xy(1) + 0.3, task_waypoints_full_sequence(i).xy(2), ...
        strrep(task_waypoints_full_sequence(i).name, '_', ' '), 'Color', 'blue', 'FontSize', 8);
end

legend(ax_main, 'off'); % Turn off old legends
legend(ax_main, 'show', 'Location', 'southoutside'); % Show a clean new one


%% --- 3. Initialize Vehicle State & Path Accumulation Variables ---

% Set the vehicle's starting pose to the first waypoint in the sequence
current_vehicle_pose_world = [task_waypoints_full_sequence(1).xy, task_waypoints_full_sequence(1).theta_rad];
prev_trailer_sim_end_theta = current_vehicle_pose_world(3); % Initial trailer angle matches truck

% Variables to store the full, concatenated path after each segment is processed
x_path_full_executed_ref = [];
y_path_full_executed_ref = [];


%% --- 4. Centralized Vehicle Definitions & Costmaps ---

% --- Vehicle Geometry and Kinematics ---
current_geom = geom();
current_geom.dt = 0.05;      % Simulation time step [s]
current_geom.v = 0.5;        % Vehicle speed [m/s]
current_geom.look = 0.7;     % Lookahead distance for Pure Pursuit [m]
current_geom.WB_truck = current_geom.Ltr * 0.70;
current_geom.WB_trailer = current_geom.Ltl * 0.70;

% --- Truck Dimensions and Collision Checker ---
RO_truck_param = current_geom.Ltr * 0.15;
WB_truck_param_for_dims = current_geom.WB_truck;
FO_truck_param = current_geom.Ltr - WB_truck_param_for_dims - RO_truck_param;
if FO_truck_param < -1e-6, FO_truck_param = max(0, FO_truck_param); end

truck_dimensions = vehicleDimensions(current_geom.Ltr, current_geom.Wtr, current_geom.Wtr, ...
    'Wheelbase', WB_truck_param_for_dims, ...
    'RearOverhang', RO_truck_param, ...
    'FrontOverhang', FO_truck_param);
truck_checker_hard = inflationCollisionChecker(truck_dimensions, 'NumCircles', 5);

% --- Trailer Dimensions and Collision Checker ---
RO_trailer_param = current_geom.Ltl * 0.15;
WB_trailer_param_for_dims = current_geom.WB_trailer;
FO_trailer_param = current_geom.Ltl - WB_trailer_param_for_dims - RO_trailer_param;
if FO_trailer_param < -1e-6, FO_trailer_param = max(0, FO_trailer_param); end

trailer_dimensions = vehicleDimensions(current_geom.Ltl, current_geom.Wtl, current_geom.Wtl, ...
    'Wheelbase', WB_trailer_param_for_dims, ...
    'RearOverhang', RO_trailer_param, ...
    'FrontOverhang', FO_trailer_param);
trailer_checker_hard = inflationCollisionChecker(trailer_dimensions, 'NumCircles', 3);

% --- Costmaps for Collision Checking ---
costmap_truck_hard = vehicleCostmap(environmentMap, 'CollisionChecker', truck_checker_hard);
costmap_trailer_hard = vehicleCostmap(environmentMap, 'CollisionChecker', trailer_checker_hard);

% Create additional costmaps for detecting "close calls" (proximity warnings)
CLOSE_CALL_THRESHOLD_VINEYARD = 0.15; % meters
costmap_truck_close = [];
costmap_trailer_close = [];
if CLOSE_CALL_THRESHOLD_VINEYARD > 0
    inflated_map_for_close_calls = copy(environmentMap);
    inflate(inflated_map_for_close_calls, CLOSE_CALL_THRESHOLD_VINEYARD);
    costmap_truck_close = vehicleCostmap(inflated_map_for_close_calls, 'CollisionChecker', truck_checker_hard);
    costmap_trailer_close = vehicleCostmap(inflated_map_for_close_calls, 'CollisionChecker', trailer_checker_hard);
end

% Package costmaps into a struct for easy passing to functions
app_costmaps = struct(...
    'truck_hard', costmap_truck_hard, ...
    'trailer_hard', costmap_trailer_hard, ...
    'truck_close', costmap_truck_close, ...
    'trailer_close', costmap_trailer_close ...
);

fprintf('Vehicle definitions and costmaps created.\n');


%% ------------------------------------------------------------------------
%                           MAIN EXECUTION LOOP
%  ------------------------------------------------------------------------

h_plots_on_main_ax = gobjects(0); % Handles for plots to clear each iteration

for seg_idx = 1:(length(task_waypoints_full_sequence) - 1)

    % --- A. Get Start and End Waypoints for the Current Segment ---
    wp_start_details = task_waypoints_full_sequence(seg_idx);
    wp_end_details = task_waypoints_full_sequence(seg_idx + 1);

    fprintf('\n--- Segment %d: From %s (%s) to %s (%s) ---\n', ...
        seg_idx, wp_start_details.name, wp_start_details.waypoint_type, ...
        wp_end_details.name, wp_end_details.waypoint_type);
    fprintf('Vehicle start: X=%.2f, Y=%.2f, Th=%.2f rad. PrevTrTh=%.2f\n', ...
        current_vehicle_pose_world(1), current_vehicle_pose_world(2), ...
        current_vehicle_pose_world(3), prev_trailer_sim_end_theta);

    % Clear previous segment's plots from the main axis
    if ~isempty(h_plots_on_main_ax)
        valid_h = isgraphics(h_plots_on_main_ax);
        if any(valid_h)
            delete(h_plots_on_main_ax(valid_h));
        end
    end
    h_plots_on_main_ax = gobjects(0);

    % The actual starting pose for this segment is the final pose from the last one
    actual_vehicle_start_pose_for_segment_xy_th = current_vehicle_pose_world;

    % --- B. Generate the Initial Reference Path for the Segment ---
    [x_seg_ref, y_seg_ref, theta_seg_ref, segment_type, h_bezier_debug_plots_on_main] = ...
        generateSingleSegmentPath_V2(wp_start_details, actual_vehicle_start_pose_for_segment_xy_th, ...
        wp_end_details, UPDATED_VINEYARD_PARAMS, ax_main, ...
        planningMap, prm_nodes_initial, prm_adjacency_initial);

    % Store handles of any debug plots created during path generation
    h_plots_on_main_ax = [h_plots_on_main_ax, h_bezier_debug_plots_on_main(isgraphics(h_bezier_debug_plots_on_main))];

    % Check if path generation was successful
    if isempty(x_seg_ref) || length(x_seg_ref) < 2
        fprintf('Seg %d path generation failed or path is too short. Skipping segment.\n', seg_idx);
        %input('Error occurred. Press Enter to continue...');
        continue;
    end
    
    fprintf('INFO: Seg %d (%s): Start(X=%.1f,Y=%.1f), End(X=%.1f,Y=%.1f), Target(X=%.1f,Y=%.1f). Pts:%d\n', ...
        seg_idx, segment_type, x_seg_ref(1), y_seg_ref(1), x_seg_ref(end), y_seg_ref(end), ...
        wp_end_details.xy(1), wp_end_details.xy(2), length(x_seg_ref));
    
    % Plot the generated reference path
    h_current_segment_plot = plot(ax_main, x_seg_ref, y_seg_ref, 'c--', 'LineWidth', 1.5, ...
        'DisplayName', 'Current Segment (Initial Ref)');
    h_plots_on_main_ax(end+1) = h_current_segment_plot;
    legend(ax_main, 'off'); legend(ax_main, 'show', 'Location', 'southoutside');
    
   % input(sprintf('Seg %d ref path generated. Press Enter to simulate initial path...', seg_idx), 's');

    % --- C. Simulate the Vehicle Following the INITIAL Reference Path ---
    
    % Set initial vehicle states for the simulation
    x0_t = actual_vehicle_start_pose_for_segment_xy_th(1);
    y0_t = actual_vehicle_start_pose_for_segment_xy_th(2);
    th0_t_rad = actual_vehicle_start_pose_for_segment_xy_th(3);
    th0_tr_rad = prev_trailer_sim_end_theta;
    
    % Calculate initial trailer position based on truck and hitch geometry
    hitch_x_curr = x0_t - (current_geom.Ltr / 2) * cos(th0_t_rad);
    hitch_y_curr = y0_t - (current_geom.Ltr / 2) * sin(th0_t_rad);
    dist_h_to_tr_geom = current_geom.Lh + (current_geom.Ltl / 2);
    x0_tr = hitch_x_curr - dist_h_to_tr_geom * cos(th0_tr_rad);
    y0_tr = hitch_y_curr - dist_h_to_tr_geom * sin(th0_tr_rad);
    
    % Run the simulation
    [truck_sim_seg_orig, trailer_sim_seg_orig, target_xs_orig_seg, target_ys_orig_seg] = ...
        simulate_vehicle_from_pose(x_seg_ref, y_seg_ref, x0_t, y0_t, th0_t_rad, x0_tr, y0_tr, th0_tr_rad, ...
        current_geom, RO_truck_param, RO_trailer_param, segment_type);

    % Animate the result of the initial simulation
    % if ~isempty(truck_sim_seg_orig.x)
    %     animate_vehicles_with_map(environmentMap, x_seg_ref, y_seg_ref, ...
    %         truck_sim_seg_orig, trailer_sim_seg_orig, target_xs_orig_seg, target_ys_orig_seg);
    %     anim_fig_h = findall(0, 'Type', 'figure', '-regexp', 'Name', 'Truck-Trailer on Open Field');
    %     if ~isempty(anim_fig_h)
    %         title(anim_fig_h(1).CurrentAxes, sprintf('Anim: Seg %d (Initial): %s to %s', ...
    %             seg_idx, strrep(wp_start_details.name, '_', ' '), strrep(wp_end_details.name, '_', ' ')));
    %     end
    % else
    %     fprintf('Skipping animation: initial sim for seg %d was empty.\n', seg_idx);
    % end
   % input(sprintf('Seg %d initial animation complete. Press Enter to proceed with modification...', seg_idx), 's');
    
% --- D. Modify the Path (if it's a candidate curve type) and Re-simulate ---
    
    % By default, the executed path and its simulation are the original ones
    x_seg_executed = x_seg_ref;
    y_seg_executed = y_seg_ref;
    theta_seg_executed = theta_seg_ref; % This will be updated if x_seg_executed changes
    
    truck_sim_seg_final = truck_sim_seg_orig;
    trailer_sim_seg_final = trailer_sim_seg_orig;
    target_xs_final_seg = target_xs_orig_seg;
    target_ys_final_seg = target_ys_orig_seg;
    
    RUN_MODIFY_CURVES = true; % Master switch, keep true to test modification
    
    % Determine if this segment type should be considered for modification
    is_candidate_for_modification = strcmp(segment_type, 'turn_bezier') || ...
                                    strcmp(segment_type, 'prm_custom') || ...
                                    strcmp(segment_type, 'prm_default');
                                    
    can_run_modification = ~isempty(truck_sim_seg_orig.x) && RUN_MODIFY_CURVES; % Ensure initial sim was valid
                         
    if is_candidate_for_modification && can_run_modification
        fprintf('Segment %d (type: %s) is a candidate. Applying modify_curves_bezier...\n', seg_idx, segment_type);
        
        % --- D1. Define Curve Bounds for modify_curves_bezier ---
        % For segment-wise processing, we treat the entire current segment (x_seg_ref)
        % as the "curve" to be analyzed and potentially modified.
        % We provide minimal padding as modify_curves_bezier handles its own context.
        
        num_points_in_segment = length(x_seg_ref);
        padding_for_modify = 2; % Minimal padding, or even 0 if modify_curves_bezier handles endpoints well.
                                % This gives it a tiny bit of context before/after if its internal logic needs it.
                                
        curve_start_idx_for_modify = max(1, 1 - padding_for_modify); % Effectively 1
        curve_end_idx_for_modify   = min(num_points_in_segment, num_points_in_segment + padding_for_modify); % Effectively num_points_in_segment
        
        % The 3rd element of curve_bounds is max_omega, can be a dummy value if modify_curves_bezier does not use it
        % Or calculate it for completeness:
        max_omega_in_seg = 0;
        if num_points_in_segment >= 2
            local_dt_temp = current_geom.dt;
            dtheta_loc_temp = diff(unwrap(theta_seg_ref));
            omega_loc_temp = [0, dtheta_loc_temp/local_dt_temp];
            if ~isempty(omega_loc_temp)
                max_omega_in_seg = max(abs(omega_loc_temp(isfinite(omega_loc_temp))));
            end
        end
        
        % Define a single curve encompassing the (potentially padded) segment
        curve_bounds_to_pass = [curve_start_idx_for_modify, curve_end_idx_for_modify, max_omega_in_seg]; 
        
        fprintf('  Passing curve bounds [%d, %d] (relative to current segment) to modify_curves_bezier.\n', curve_bounds_to_pass(1), curve_bounds_to_pass(2));

        % --- D2. Apply Bezier Modification using modify_curves_bezier ---
        current_geom_for_modify = current_geom; % Contains v, dt, WB_truck etc.
        % Global hotspots are not passed per segment here, assuming local evaluation is primary.
        % If modify_curves_bezier *requires* global_hotspots from a full prior sim, this would need to change.
        
        [x_seg_mod_temp, y_seg_mod_temp, overall_modification_successful_seg] = ...
            modify_curves_bezier(x_seg_ref, y_seg_ref, theta_seg_ref, curve_bounds_to_pass, ...
            environmentMap, truck_sim_seg_orig, trailer_sim_seg_orig, current_geom_for_modify, ...
            RO_truck_param, RO_trailer_param, app_costmaps, truck_dimensions, trailer_dimensions, []); % Passing empty global_hotspots

        if overall_modification_successful_seg && ~isempty(x_seg_mod_temp) && length(x_seg_mod_temp) >= 2
            % Modification was successful and returned a valid path
            x_seg_executed = x_seg_mod_temp; % This is now the path to be simulated
            y_seg_executed = y_seg_mod_temp;
            
            % Recalculate theta for the modified segment
            dx_mod = diff(x_seg_executed);
            dy_mod = diff(y_seg_executed);
            theta_calc_mod = atan2(dy_mod, dx_mod);
            theta_seg_executed = [theta_calc_mod(:)', theta_calc_mod(end)]; % Ensure row vector and replicate last
            
            fprintf('Seg %d modify_curves_bezier successful. Plotting modified reference path.\n', seg_idx);
            h_mod_plot = plot(ax_main, x_seg_executed, y_seg_executed, 'g-', 'LineWidth', 2, ...
                'DisplayName', 'Segment (Ref Modified)');
            h_plots_on_main_ax(end+1) = h_mod_plot; % Add to list of plots on main axes
            legend(ax_main, 'off'); legend(ax_main, 'show'); % Refresh legend
        else
            fprintf('!!! Seg %d modify_curves_bezier FAILED or returned too short path. Using original reference path. !!!\n', seg_idx);
            % x_seg_executed and theta_seg_executed remain as the original x_seg_ref, theta_seg_ref
            % The simulation will proceed with the original path for this segment.
        end
        
    elseif RUN_MODIFY_CURVES == false && is_candidate_for_modification
         fprintf('Skipped modify_curves_bezier for segment %d (type: %s) because RUN_MODIFY_CURVES=false.\n', seg_idx, segment_type);
    elseif ~is_candidate_for_modification && RUN_MODIFY_CURVES
        fprintf('Segment %d (type: %s) is not a candidate for modification. Using initial path.\n', seg_idx, segment_type);
    elseif ~can_run_modification && is_candidate_for_modification && RUN_MODIFY_CURVES
        fprintf('Skipped modify_curves_bezier for segment %d: initial simulation was empty or RUN_MODIFY_CURVES is false.\n', seg_idx);
    end
    
    % --- E. FINAL Simulation of the Executed Path (either original or modified) ---
    % It will use x_seg_executed, y_seg_executed, theta_seg_executed for the final simulation.
    
    % --- E. FINAL Simulation of the Executed Path ---
    fprintf('Simulating FINAL EXECUTED path for segment %d...\n', seg_idx);
    
    [truck_sim_seg_final, trailer_sim_seg_final, target_xs_final_seg, target_ys_final_seg] = ...
        simulate_vehicle_from_pose(x_seg_executed, y_seg_executed, x0_t, y0_t, th0_t_rad, ...
        x0_tr, y0_tr, th0_tr_rad, current_geom, RO_truck_param, RO_trailer_param, segment_type);

    % Animate the final executed path
    if ~isempty(truck_sim_seg_final.x)
        animate_vehicles_with_map(environmentMap, x_seg_executed, y_seg_executed, ...
            truck_sim_seg_final, trailer_sim_seg_final, target_xs_final_seg, target_ys_final_seg);
        anim_fig_h = findall(0, 'Type', 'figure', '-regexp', 'Name', 'Truck-Trailer on Open Field');
        if ~isempty(anim_fig_h)
            title(anim_fig_h(1).CurrentAxes, sprintf('Anim: Seg %d (Final Executed): %s to %s', ...
                seg_idx, strrep(wp_start_details.name, '_', ' '), strrep(wp_end_details.name, '_', ' ')));
        end
    else
        fprintf('Skipping animation: final simulation for seg %d was empty.\n', seg_idx);
    end
    %input(sprintf('Seg %d final executed animation complete. Press Enter to proceed to next segment...', seg_idx), 's');
    
    % --- F. Accumulate Full Path and Update Vehicle State for Next Loop ---
    
    % Add the executed segment to the full path history
    current_segment_x_to_add = x_seg_executed(:)';
    current_segment_y_to_add = y_seg_executed(:)';
    if isempty(x_path_full_executed_ref)
        x_path_full_executed_ref = current_segment_x_to_add;
        y_path_full_executed_ref = current_segment_y_to_add;
    else
        % Avoid duplicating the connection point
        if length(current_segment_x_to_add) > 1
            x_path_full_executed_ref = [x_path_full_executed_ref(:)', current_segment_x_to_add(2:end)];
            y_path_full_executed_ref = [y_path_full_executed_ref(:)', current_segment_y_to_add(2:end)];
        elseif length(current_segment_x_to_add) == 1 && ~isempty(x_path_full_executed_ref)
            % Handle single-point segments if they are not redundant
            if norm([x_path_full_executed_ref(end), y_path_full_executed_ref(end)] - ...
                    [current_segment_x_to_add, current_segment_y_to_add]) > 1e-3
                x_path_full_executed_ref = [x_path_full_executed_ref(:)', current_segment_x_to_add];
                y_path_full_executed_ref = [y_path_full_executed_ref(:)', current_segment_y_to_add];
            end
        elseif length(current_segment_x_to_add) == 1 && isempty(x_path_full_executed_ref)
             x_path_full_executed_ref = current_segment_x_to_add;
             y_path_full_executed_ref = current_segment_y_to_add;
        end
    end
    
    % Update the vehicle's state based on the end of the simulation
    last_valid_truck_sim = [];
    last_valid_trailer_sim = [];
    if ~isempty(truck_sim_seg_final) && isfield(truck_sim_seg_final, 'x') && ~isempty(truck_sim_seg_final.x)
        last_valid_truck_sim = truck_sim_seg_final;
        last_valid_trailer_sim = trailer_sim_seg_final;
    elseif ~isempty(truck_sim_seg_orig) && isfield(truck_sim_seg_orig, 'x') && ~isempty(truck_sim_seg_orig.x)
        % Fallback to original simulation if final one failed
        last_valid_truck_sim = truck_sim_seg_orig;
        last_valid_trailer_sim = trailer_sim_seg_orig;
    end
    
    if ~isempty(last_valid_truck_sim)
        current_vehicle_pose_world = [last_valid_truck_sim.x(end), ...
                                      last_valid_truck_sim.y(end), ...
                                      last_valid_truck_sim.theta(end)];
        if ~isempty(last_valid_trailer_sim) && isfield(last_valid_trailer_sim, 'theta') && ~isempty(last_valid_trailer_sim.theta)
            prev_trailer_sim_end_theta = last_valid_trailer_sim.theta(end);
        else
            prev_trailer_sim_end_theta = current_vehicle_pose_world(3); % Fallback
            fprintf('Warning: Trailer simulation theta was empty at end of segment %d.\n', seg_idx);
        end
    else
        fprintf('CRITICAL WARNING: No valid simulation data for segment %d. Updating pose from geometric path end.\n', seg_idx);
        if ~isempty(theta_seg_executed)
            current_vehicle_pose_world = [x_seg_executed(end), y_seg_executed(end), theta_seg_executed(end)];
            prev_trailer_sim_end_theta = theta_seg_executed(end);
        elseif ~isempty(theta_seg_ref)
             current_vehicle_pose_world = [x_seg_executed(end), y_seg_executed(end), theta_seg_ref(end)];
             prev_trailer_sim_end_theta = theta_seg_ref(end);
        else
             current_vehicle_pose_world = [x_seg_executed(end), y_seg_executed(end), current_vehicle_pose_world(3)];
             prev_trailer_sim_end_theta = current_vehicle_pose_world(3);
        end
    end
    
    fprintf('Vehicle pose after seg %d: X=%.2f, Y=%.2f, Th=%.2f rad. PrevTrTh=%.2f\n', ...
        seg_idx, current_vehicle_pose_world(1), current_vehicle_pose_world(2), ...
        current_vehicle_pose_world(3), prev_trailer_sim_end_theta);
    
end % End of main execution loop


%% ------------------------------------------------------------------------
%                                  EPILOGUE
%  ------------------------------------------------------------------------

fprintf('\n--- All Segments Processed ---\n');

% Plot the final, complete path on the main figure
figure(fig_main_handle);
plot(ax_main, x_path_full_executed_ref, y_path_full_executed_ref, 'm.-', ...
    'LineWidth', 1.2, 'DisplayName', 'Full Planned Path (Concatenated Ref Segments)');
legend(ax_main, 'off');
legend(ax_main, 'show', 'Location', 'southoutside');
title(ax_main, 'Full Path Execution Plan');


%% ------------------------------------------------------------------------
%                           LOCAL HELPER FUNCTIONS
%  ------------------------------------------------------------------------

function VINEYARD_PARAMS = defineVineyardParameters()
    % defineVineyardParameters Defines the geometric properties of the vineyard.
    %
    %   Outputs:
    %       VINEYARD_PARAMS: (struct) A structure containing all geometric data.
    
    VINEYARD_PARAMS.field_x_limits = [1, 49];
    VINEYARD_PARAMS.bottom_headland_y_limits = [2.3, 6];
    VINEYARD_PARAMS.top_headland_y_limits = [44.5, 48];
    
    % Define the y-coordinates for row entry/exit points
    VINEYARD_PARAMS.row_y_start = VINEYARD_PARAMS.bottom_headland_y_limits(2) + 0.5;
    VINEYARD_PARAMS.row_y_end = VINEYARD_PARAMS.top_headland_y_limits(1) - 0.5;
    VINEYARD_PARAMS.row_y_bottom_snap = VINEYARD_PARAMS.bottom_headland_y_limits(2) + 0.5;
    VINEYARD_PARAMS.row_y_top_snap = VINEYARD_PARAMS.top_headland_y_limits(1) - 0.5;
    
    % Define row centers
    row_centers_x = [3, 7, 10.75]; % Adjusted Row 3 center slightly for filari.png
    VINEYARD_PARAMS.rows = [];
    for i = 1:length(row_centers_x)
        VINEYARD_PARAMS.rows(i).name = sprintf('Row_%d', i);
        VINEYARD_PARAMS.rows(i).center_x = row_centers_x(i);
        
        % Define the 4 standard snap points for each row
        VINEYARD_PARAMS.rows(i).bottom_entry_xy = [row_centers_x(i), VINEYARD_PARAMS.row_y_bottom_snap];
        VINEYARD_PARAMS.rows(i).bottom_entry_theta_rad = pi/2;
        
        VINEYARD_PARAMS.rows(i).top_exit_xy = [row_centers_x(i), VINEYARD_PARAMS.row_y_top_snap];
        VINEYARD_PARAMS.rows(i).top_exit_theta_rad = pi/2;
        
        VINEYARD_PARAMS.rows(i).top_entry_xy = [row_centers_x(i), VINEYARD_PARAMS.row_y_top_snap];
        VINEYARD_PARAMS.rows(i).top_entry_theta_rad = -pi/2;
        
        VINEYARD_PARAMS.rows(i).bottom_exit_xy = [row_centers_x(i), VINEYARD_PARAMS.row_y_bottom_snap];
        VINEYARD_PARAMS.rows(i).bottom_exit_theta_rad = -pi/2;
    end
    
    if length(VINEYARD_PARAMS.rows) >= 2
        VINEYARD_PARAMS.row_width_center_to_center = VINEYARD_PARAMS.rows(2).center_x - VINEYARD_PARAMS.rows(1).center_x;
    else
        VINEYARD_PARAMS.row_width_center_to_center = 4; % Default value
    end
    
    VINEYARD_PARAMS.parking_spots = []; %non utilizzata
end

% -------------------------------------------------------------------------

function all_snap_points = collectAllSnapPoints(VINEYARD_PARAMS)
    % collectAllSnapPoints Gathers all defined snap points into a single struct array.
    %
    %   Inputs:
    %       VINEYARD_PARAMS: (struct) Vineyard parameters structure.
    %   Outputs:
    %       all_snap_points: (struct array) A list of all snap points.
    
    all_snap_points = struct('name', {}, 'xy', {}, 'theta_rad', {});
    
    % Add row-related snap points
    for i = 1:length(VINEYARD_PARAMS.rows)
        row_name = VINEYARD_PARAMS.rows(i).name;
        all_snap_points(end+1) = struct('name', [row_name '_BottomEntry'], 'xy', VINEYARD_PARAMS.rows(i).bottom_entry_xy, 'theta_rad', VINEYARD_PARAMS.rows(i).bottom_entry_theta_rad);
        all_snap_points(end+1) = struct('name', [row_name '_TopExit'],    'xy', VINEYARD_PARAMS.rows(i).top_exit_xy,    'theta_rad', VINEYARD_PARAMS.rows(i).top_exit_theta_rad);
        all_snap_points(end+1) = struct('name', [row_name '_TopEntry'],    'xy', VINEYARD_PARAMS.rows(i).top_entry_xy,    'theta_rad', VINEYARD_PARAMS.rows(i).top_entry_theta_rad);
        all_snap_points(end+1) = struct('name', [row_name '_BottomExit'],  'xy', VINEYARD_PARAMS.rows(i).bottom_exit_xy,  'theta_rad', VINEYARD_PARAMS.rows(i).bottom_exit_theta_rad);
    end
    
    % Add parking-related snap points (if they exist)
    if isfield(VINEYARD_PARAMS, 'parking_spots') && ~isempty(VINEYARD_PARAMS.parking_spots)
        for i = 1:length(VINEYARD_PARAMS.parking_spots)
            all_snap_points(end+1) = struct(...
                'name', VINEYARD_PARAMS.parking_spots(i).name, ...
                'xy', VINEYARD_PARAMS.parking_spots(i).coords_xy, ...
                'theta_rad', VINEYARD_PARAMS.parking_spots(i).theta_rad);
        end
    end
end

% -------------------------------------------------------------------------

function waypoints = getWaypointsFromMixedTask(task_sequence_cell_array, all_snap_points, all_custom_points)
    % getWaypointsFromMixedTask Converts a high-level task list into a waypoint struct.
    %
    %   Inputs:
    %       task_sequence_cell_array: (cell) High-level list of tasks.
    %       all_snap_points: (struct array) Available snap points.
    %       all_custom_points: (struct array) Available custom points.
    %   Outputs:
    %       waypoints: (struct array) The final sequence of waypoints with details.
    
    waypoints = struct('name', {}, 'xy', {}, 'theta_rad', {}, 'waypoint_type', {}, 'snap_type', {}, 'theta_rad_implicit_exit', {});
    if isempty(task_sequence_cell_array)
        warning('Task sequence is empty.');
        return;
    end
    
    for i = 1:length(task_sequence_cell_array)
        task_item = task_sequence_cell_array{i};
        item_type = task_item{2};
        item_name = task_item{4};
        
        waypoints(i).waypoint_type = item_type;
        waypoints(i).name = item_name;
        
        if strcmp(item_type, 'snap')
            idx = find(strcmp({all_snap_points.name}, item_name));
            if isempty(idx), error('Snap point name "%s" not found.', item_name); end
            waypoints(i).xy = all_snap_points(idx).xy;
            waypoints(i).theta_rad = all_snap_points(idx).theta_rad;
            
            % Categorize the snap point type
            if contains(item_name, 'Parking'), waypoints(i).snap_type = 'parking';
            elseif contains(item_name, 'Entry'), waypoints(i).snap_type = 'row_entry';
            elseif contains(item_name, 'Exit'), waypoints(i).snap_type = 'row_exit';
            else, waypoints(i).snap_type = 'unknown_snap';
            end
            
        elseif strcmp(item_type, 'custom')
            idx = find(strcmp({all_custom_points.name}, item_name));
            if isempty(idx), error('Custom point name "%s" not found.', item_name); end
            waypoints(i).xy = all_custom_points(idx).xy;
            waypoints(i).theta_rad = all_custom_points(idx).theta_rad;
            waypoints(i).snap_type = 'custom';
            
        else
            error('Unknown waypoint type: %s', item_type);
        end
    end
    
    % Calculate the implicit exit orientation for each waypoint (pointing to the next)
    for i = 1:(length(waypoints) - 1)
        p_start = waypoints(i).xy;
        p_end = waypoints(i+1).xy;
        waypoints(i).theta_rad_implicit_exit = atan2(p_end(2) - p_start(2), p_end(1) - p_start(1));
    end
    
    % The last waypoint's exit angle is its own defined orientation
    if ~isempty(waypoints)
        waypoints(end).theta_rad_implicit_exit = waypoints(end).theta_rad;
    end
end

% -------------------------------------------------------------------------

function [x_seg, y_seg, theta_seg, segment_type_str, h_bezier_debug_plots] = generateSingleSegmentPath_V2(...
    wp_start_details, actual_vehicle_start_pose_xy_theta, wp_end_details, ...
    VINEYARD_PARAMS, ax_for_debug_plot, prm_planning_map, prm_nodes, prm_adjacency)
    % generateSingleSegmentPath_V2 Chooses a path generation strategy based on segment type.
    
    h_bezier_debug_plots = gobjects(0);
    
    % Extract start and end goals
    P0_actual = actual_vehicle_start_pose_xy_theta(1:2);
    Theta0_actual = actual_vehicle_start_pose_xy_theta(3);
    P3_target_xy = wp_end_details.xy;
    Theta3_target_orientation = wp_end_details.theta_rad;
    
    % --- Determine Segment Type ---
    segment_type_str = 'prm_default'; % Default to PRM for unknown transitions
    start_is_snap = strcmp(wp_start_details.waypoint_type, 'snap');
    end_is_snap = strcmp(wp_end_details.waypoint_type, 'snap');
    
    if start_is_snap && end_is_snap
        % Check for in-row straight segment
        if (strcmp(wp_start_details.snap_type, 'row_entry') && strcmp(wp_end_details.snap_type, 'row_exit'))
            start_row_parts = strsplit(wp_start_details.name, '_');
            end_row_parts = strsplit(wp_end_details.name, '_');
            if strcmp(start_row_parts{2}, end_row_parts{2}) % e.g., Row_1_... to Row_1_...
                segment_type_str = 'in_row';
            end
        % Check for headland turn segment
        elseif (strcmp(wp_start_details.snap_type, 'row_exit') && strcmp(wp_end_details.snap_type, 'row_entry'))
            segment_type_str = 'turn_bezier';
        end
    end
    
    % If either start or end is a custom point, use PRM
    if strcmp(wp_start_details.waypoint_type, 'custom') || strcmp(wp_end_details.waypoint_type, 'custom')
        segment_type_str = 'prm_custom';
    end
    
    % --- Generate Path Based on Type ---
    x_seg = []; y_seg = []; theta_seg = [];
    points_per_meter_straight = 20;
    points_per_meter_turn = 20;
    points_per_meter_prm = 10;
    
    switch segment_type_str
        case 'in_row'
            dist_seg = norm(P3_target_xy - P0_actual);
            num_pts = max(2, ceil(dist_seg * points_per_meter_straight));
            x_seg = linspace(P0_actual(1), P3_target_xy(1), num_pts);
            y_seg = linspace(P0_actual(2), P3_target_xy(2), num_pts);
            
        case 'turn_bezier'
            % Determine which headland (top or bottom) the turn is in
            if P0_actual(2) > (VINEYARD_PARAMS.bottom_headland_y_limits(2) + VINEYARD_PARAMS.top_headland_y_limits(1)) / 2
                headland_lims = VINEYARD_PARAMS.top_headland_y_limits;
            else
                headland_lims = VINEYARD_PARAMS.bottom_headland_y_limits;
            end
            headland_depth = abs(headland_lims(2) - headland_lims(1));
            
            [x_seg, y_seg, ~, h_bezier_debug_plots] = generateInitialHeadlandBezier_V2(...
                P0_actual, Theta0_actual, P3_target_xy, Theta3_target_orientation, ...
                VINEYARD_PARAMS, headland_depth, points_per_meter_turn, ax_for_debug_plot);
                
        case {'prm_default', 'prm_custom'}
            fprintf('  Using PRM for segment: %s to %s\n', wp_start_details.name, wp_end_details.name);
            [path_nodes_prm, success] = findPathPRM(P0_actual, P3_target_xy, ...
                prm_nodes, prm_adjacency, prm_planning_map);
            
            if success && ~isempty(path_nodes_prm)
                if size(path_nodes_prm, 1) == 1
                    x_seg = path_nodes_prm(1, 1);
                    y_seg = path_nodes_prm(1, 2);
                else
                    % Interpolate along the PRM path for a smooth trajectory
                    path_dist = [0; cumsum(sqrt(sum(diff(path_nodes_prm, 1, 1).^2, 2)))];
                    num_interp = max(2, ceil(path_dist(end) * points_per_meter_prm));
                    interp_dist = linspace(0, path_dist(end), num_interp);
                    x_seg = interp1(path_dist, path_nodes_prm(:, 1), interp_dist, 'pchip');
                    y_seg = interp1(path_dist, path_nodes_prm(:, 2), interp_dist, 'pchip');
                end
            else
                fprintf('  PRM failed. Fallback to straight line.\n');
                segment_type_str = 'straight_fallback_prm';
                dist_seg = norm(P3_target_xy - P0_actual);
                num_pts = max(2, ceil(dist_seg * points_per_meter_straight));
                x_seg = linspace(P0_actual(1), P3_target_xy(1), num_pts);
                y_seg = linspace(P0_actual(2), P3_target_xy(2), num_pts);
            end
            
        otherwise
            fprintf('  Unknown segment type. Fallback to straight line.\n');
            segment_type_str = 'straight_fallback_unknown';
            dist_seg = norm(P3_target_xy - P0_actual);
            num_pts = max(2, ceil(dist_seg * points_per_meter_straight));
            x_seg = linspace(P0_actual(1), P3_target_xy(1), num_pts);
            y_seg = linspace(P0_actual(2), P3_target_xy(2), num_pts);
    end
    
    % Calculate path orientation (theta) from the generated [x, y] points
    if ~isempty(x_seg) && length(x_seg) > 1
        dx = diff(x_seg);
        dy = diff(y_seg);
        th_calc = atan2(dy, dx);
        theta_seg = [th_calc(:)', th_calc(end)]; % Hold last angle
        theta_seg = unwrap(theta_seg);
    elseif ~isempty(x_seg) && length(x_seg) == 1
        theta_seg = Theta0_actual; % Single point path, use start orientation
    else
        theta_seg = []; x_seg = []; y_seg = []; % Path generation failed
        fprintf('Warning: Empty path for segment: %s to %s (Type: %s).\n', ...
            wp_start_details.name, wp_end_details.name, segment_type_str);
    end
end

% -------------------------------------------------------------------------

function [x_bezier, y_bezier, control_points_struct, h_debug_plots] = generateInitialHeadlandBezier_V2(...
    P0_actual, theta0_actual, P3_target, theta3_target_entry, VINEYARD_PARAMS, ...
    headland_depth, points_per_meter, ax_to_plot_on)
    % generateInitialHeadlandBezier_V2 Creates a cubic Bezier curve for headland turns.
    
    h_debug_plots = gobjects(0);
    
    % --- Tuning Factors for Bezier Control Points ---
    d_factor_longitudinal_P1 = 1.0;
    d_factor_lateral_P1 = 1.0;
    d_factor_longitudinal_P2 = 1.0;
    d_factor_lateral_P2 = 1.0;
    
    % Base distance for control points, related to row width and headland depth
    base_dist_P1 = max(VINEYARD_PARAMS.row_width_center_to_center * 0.8, headland_depth * 0.5);
    base_dist_P2 = base_dist_P1;
    
    d1_long = base_dist_P1 * d_factor_longitudinal_P1;
    d1_lat = VINEYARD_PARAMS.row_width_center_to_center * d_factor_lateral_P1;
    d2_long = base_dist_P2 * d_factor_longitudinal_P2;
    d2_lat = VINEYARD_PARAMS.row_width_center_to_center * d_factor_lateral_P2;
    
    % --- Calculate Control Points P1 and P2 ---
    dir_P0 = [cos(theta0_actual), sin(theta0_actual)];
    delta_x_turn = P3_target(1) - P0_actual(1);
    
    % Determine turn direction (left or right)
    if abs(wrapToPi(theta0_actual - pi/2)) < pi/4 % Exiting upwards
        turn_dir_k = sign(delta_x_turn);
    elseif abs(wrapToPi(theta0_actual - (-pi/2))) < pi/4 % Exiting downwards
        turn_dir_k = -sign(delta_x_turn);
    else % General case using cross product
        vec_P0_P3 = P3_target - P0_actual;
        cross_z = dir_P0(1) * vec_P0_P3(2) - dir_P0(2) * vec_P0_P3(1);
        turn_dir_k = sign(cross_z);
    end
    if turn_dir_k == 0, turn_dir_k = 1; end % Avoid zero turn direction
    
    % Calculate P1 based on start point and orientation
    perp_P0 = [-dir_P0(2) * turn_dir_k, dir_P0(1) * turn_dir_k];
    P1 = P0_actual + d1_long * dir_P0 + d1_lat * perp_P0;
    
    % Calculate P2 based on end point and orientation
    dir_P3_entry = [cos(theta3_target_entry), sin(theta3_target_entry)];
    perp_P3 = [-dir_P3_entry(2) * turn_dir_k, dir_P3_entry(1) * turn_dir_k];
    P2 = P3_target - d2_long * dir_P3_entry + d2_lat * perp_P3;
    
    control_points_struct = struct('P0', P0_actual, 'P1', P1, 'P2', P2, 'P3', P3_target);
    
    % --- Generate Bezier Curve Points ---
    est_len = norm(P1 - P0_actual) + norm(P2 - P1) + norm(P3_target - P2);
    num_bpts = max(40, ceil(est_len * points_per_meter));
    t_param = linspace(0, 1, num_bpts)';
    
    x_bezier = (1 - t_param).^3 .* P0_actual(1) + 3 .* (1 - t_param).^2 .* t_param .* P1(1) + 3 .* (1 - t_param) .* t_param.^2 .* P2(1) + t_param.^3 .* P3_target(1);
    y_bezier = (1 - t_param).^3 .* P0_actual(2) + 3 .* (1 - t_param).^2 .* t_param .* P1(2) + 3 .* (1 - t_param) .* t_param.^2 .* P2(2) + t_param.^3 .* P3_target(2);
    x_bezier = x_bezier';
    y_bezier = y_bezier';
    
    % --- Optional: Plot Debug Graphics ---
    if nargin > 7 && isgraphics(ax_to_plot_on) && any(ishandle(ax_to_plot_on))
        % Clear old debug plots
        current_plots_in_ax = findobj(ax_to_plot_on);
        debug_tags = {'dbg_P0', 'dbg_P1', 'dbg_P2', 'dbg_P3', 'dbg_L01', 'dbg_L12', 'dbg_L23'};
        for k_tg = 1:length(debug_tags)
            delete(findobj(current_plots_in_ax, 'Tag', debug_tags{k_tg}));
        end
        
        hold(ax_to_plot_on, 'on');
        h_debug_plots(1) = plot(ax_to_plot_on, P0_actual(1), P0_actual(2), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k', 'DisplayName', 'P0 (Bz Start)', 'Tag', 'dbg_P0');
        h_debug_plots(2) = plot(ax_to_plot_on, P1(1), P1(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'P1 (Bz Ctrl)', 'Tag', 'dbg_P1');
        h_debug_plots(3) = plot(ax_to_plot_on, P2(1), P2(2), 'bx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'P2 (Bz Ctrl)', 'Tag', 'dbg_P2');
        h_debug_plots(4) = plot(ax_to_plot_on, P3_target(1), P3_target(2), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', 'P3 (Bz End)', 'Tag', 'dbg_P3');
        h_debug_plots(5) = plot(ax_to_plot_on, [P0_actual(1) P1(1)], [P0_actual(2) P1(2)], 'g:', 'HandleVisibility', 'off', 'Tag', 'dbg_L01');
        h_debug_plots(6) = plot(ax_to_plot_on, [P1(1) P2(1)], [P1(2) P2(2)], 'k:', 'HandleVisibility', 'off', 'Tag', 'dbg_L12');
        h_debug_plots(7) = plot(ax_to_plot_on, [P2(1) P3_target(1)], [P2(2) P3_target(2)], 'b:', 'HandleVisibility', 'off', 'Tag', 'dbg_L23');
        
        fprintf('DEBUG Bezier CPs Plotted: P0(%.1f,%.1f) P1(%.1f,%.1f) P2(%.1f,%.1f) P3(%.1f,%.1f)\n', P0_actual, P1, P2, P3_target);
    end
end

% -------------------------------------------------------------------------

function g = geom()
    % geom Defines the physical dimensions of the truck and trailer.
    %   Outputs:
    %       g: (struct) Structure with vehicle dimensions [meters].
    
    g.Ltr = 2.20; % Truck length
    g.Wtr = 1.00; % Truck width
    g.Ltl = 1.00; % Trailer length
    g.Wtl = 1.00; % Trailer width
    g.Lh  = 1.00; % Hitch length
end

% -------------------------------------------------------------------------

function [prm_nodes, prm_adjacency] = buildPRMRoadmap(planning_map, base_map_for_occ_check, N_nodes, connection_radius_meters)
    % buildPRMRoadmap Generates a PRM by sampling nodes and connecting them.
    
    fprintf('Step 1: Sampling %d free PRM nodes...\n', N_nodes);
    xLimits = planning_map.XWorldLimits;
    yLimits = planning_map.YWorldLimits;
    
    % --- Sample Nodes ---
    prm_nodes = [];
    attempts = 0;
    maxTries = N_nodes * 20; %num max di tentativi = 10000
    while size(prm_nodes, 1) < N_nodes && attempts < maxTries
        x = rand() * (xLimits(2) - xLimits(1)) + xLimits(1); %ex se posso solo da 20 a 50, rand in un range di 30 + 20
        y = rand() * (yLimits(2) - yLimits(1)) + yLimits(1);
        attempts = attempts + 1;
        
        % Add node only if it's in a free space on both maps
        if getOccupancy(planning_map, [x, y]) == 0 && getOccupancy(base_map_for_occ_check, [x, y]) == 0
            prm_nodes(end+1, :) = [x, y];
        end
    end
    
    if size(prm_nodes, 1) < N_nodes
        warning('Only %d PRM nodes could be found out of %d requested.', size(prm_nodes, 1), N_nodes);
    end
    if size(prm_nodes, 1) < 2
        prm_nodes = []; prm_adjacency = [];
        fprintf('Insufficient PRM nodes to build a roadmap.\n');
        return;
    end
    fprintf(' Found %d PRM nodes.\n', size(prm_nodes, 1));
    
    % --- Connect Nodes (Build Adjacency Matrix) ---
    numActualNodes = size(prm_nodes, 1);
    prm_adjacency = zeros(numActualNodes);
    map_cell_size = 1 / planning_map.Resolution;
    
    for i = 1:numActualNodes
        for j = (i + 1):numActualNodes
            p1 = prm_nodes(i, :);
            p2 = prm_nodes(j, :);
            dist = norm(p1 - p2);
            
            % If nodes are within connection radius, check for a clear path between them
            if dist <= connection_radius_meters
                num_checks = max(2, ceil(dist / (map_cell_size * 0.8)));
                ix = linspace(p1(1), p2(1), num_checks);
                iy = linspace(p1(2), p2(2), num_checks);
                is_edge_collision_free = true;
                for kc = 1:num_checks
                    if getOccupancy(planning_map, [ix(kc), iy(kc)]) == 1
                        is_edge_collision_free = false;
                        break;
                    end
                end
                
                % If path is clear, add an edge to the adjacency matrix
                if is_edge_collision_free
                    prm_adjacency(i, j) = dist;
                    prm_adjacency(j, i) = dist;
                end
            end
        end
    end
    fprintf(' PRM roadmap construction complete.\n');
end

% -------------------------------------------------------------------------

function [path_coords, success] = findPathPRM(start_coords_xy, goal_coords_xy, existing_nodes, existing_adjacency, planning_map)
    % findPathPRM Finds the shortest path in a PRM graph using Dijkstra's algorithm.
    
    path_coords = [];
    success = false;
    map_cell_size = 1 / planning_map.Resolution;
    
    % --- Validate Start and Goal ---
    if getOccupancy(planning_map, start_coords_xy) == 1
        fprintf('PRM Error: Start point is inside an obstacle!\n'); return;
    end
    if getOccupancy(planning_map, goal_coords_xy) == 1
        fprintf('PRM Error: Goal point is inside an obstacle!\n'); return;
    end
    
    % --- Build Temporary Graph with Start and Goal Nodes ---
    % [start; goal; existing_node_1; ...]
    curr_nodes = [start_coords_xy; goal_coords_xy; existing_nodes];
    numCurrNodes = size(curr_nodes, 1);
    curr_adj = zeros(numCurrNodes);
    
    % Copy existing adjacency matrix
    if ~isempty(existing_adjacency) && all(size(existing_adjacency) == size(existing_nodes, 1))
        curr_adj(3:end, 3:end) = existing_adjacency;
    elseif ~isempty(existing_adjacency)
        warning('PRM Adjacency matrix size mismatch during pathfinding.');
    end
    
    % Connect start/goal nodes to the existing graph
    conn_rad_sg = 25; % Connection radius for start/goal
    for i = 3:numCurrNodes % For each existing node
        for j = 1:2 % For start (1) and goal (2)
            dist = norm(curr_nodes(i, :) - curr_nodes(j, :));
            if dist <= conn_rad_sg
                % Check for collision-free edge
                num_c = max(2, ceil(dist / (map_cell_size * 0.8)));
                ix = linspace(curr_nodes(i, 1), curr_nodes(j, 1), num_c);
                iy = linspace(curr_nodes(i, 2), curr_nodes(j, 2), num_c);
                is_ecf = true;
                for kc = 1:num_c
                    if getOccupancy(planning_map, [ix(kc), iy(kc)]) == 1, is_ecf = false; break; end
                end
                if is_ecf, curr_adj(i, j) = dist; curr_adj(j, i) = dist; end
            end
        end
    end
    
    % Check for direct line-of-sight between start and goal
    dist_sg = norm(start_coords_xy - goal_coords_xy);
    if dist_sg <= conn_rad_sg * 2
        num_c = max(2, ceil(dist_sg / (map_cell_size * 0.8)));
        ix = linspace(start_coords_xy(1), goal_coords_xy(1), num_c);
        iy = linspace(start_coords_xy(2), goal_coords_xy(2), num_c);
        is_ecf = true;
        for kc = 1:num_c
            if getOccupancy(planning_map, [ix(kc), iy(kc)]) == 1, is_ecf = false; break; end
        end
        if is_ecf, curr_adj(1, 2) = dist_sg; curr_adj(2, 1) = dist_sg; end
    end
    
    % --- Run Dijkstra's Algorithm ---
    sIdx = 1; gIdx = 2;
    visited = false(1, numCurrNodes);
    dist_node = inf(1, numCurrNodes);
    prev_node = nan(1, numCurrNodes);
    dist_node(sIdx) = 0;
    
    for kd = 1:numCurrNodes
        tmp_dist = dist_node;
        tmp_dist(visited) = inf;
        [min_v, u] = min(tmp_dist);
        
        if isinf(min_v) || u == 0, fprintf('PRM Dijkstra: No path found (unreachable graph).\n'); return; end
        
        visited(u) = true;
        if u == gIdx, break; end % Goal reached
        
        neigh_idx = find(curr_adj(u, :) > 0);
        for v = neigh_idx
            if ~visited(v)
                alt_d = dist_node(u) + curr_adj(u, v);
                if alt_d < dist_node(v)
                    dist_node(v) = alt_d;
                    prev_node(v) = u;
                end
            end
        end
    end
    
    % --- Reconstruct Path ---
    if isinf(dist_node(gIdx)), fprintf('PRM Dijkstra: No path to goal node.\n'); return; end
    
    path_idx_rec = gIdx;
    if all(isnan(prev_node)) && gIdx ~= sIdx
        fprintf('PRM Path Reconstruction Error: prev_node array is all NaNs.\n'); return;
    end
    
    while ~isnan(prev_node(path_idx_rec(1))) && prev_node(path_idx_rec(1)) ~= 0 && prev_node(path_idx_rec(1)) ~= sIdx
        path_idx_rec = [prev_node(path_idx_rec(1)), path_idx_rec];
        if length(path_idx_rec) > numCurrNodes * 2
            warning('PRM path reconstruction seems to be in a loop.');
            path_coords = []; success = false; return;
        end
    end
    
    if ~isempty(path_idx_rec) && ~isnan(prev_node(path_idx_rec(1))) && prev_node(path_idx_rec(1)) == sIdx
        path_idx_rec = [sIdx, path_idx_rec];
        path_coords = curr_nodes(path_idx_rec, :);
        success = true;
        fprintf('PRM Path found with %d segments.\n', length(path_idx_rec) - 1);
    elseif sIdx == gIdx
        path_coords = curr_nodes(sIdx, :); success = true;
    else
        fprintf('PRM path reconstruction failed.\n');
        path_coords = []; success = false;
    end
end

% -------------------------------------------------------------------------

function [truck_s, trailer_s, target_xs_out, target_ys_out] = simulate_vehicle_from_pose(...
    xr_s, yr_s, ...
    x0_truck_geom, y0_truck_geom, th0_truck_geom_rad, ...
    x0_trailer_geom, y0_trailer_geom, th0_trailer_geom_rad, ...
    g_geom, RO_truck_param, RO_trailer_param, segment_type_str)
    % simulate_vehicle_from_pose Simulates vehicle motion using a Pure Pursuit controller.
    
    % --- Initial Checks ---
    n_s_ref = length(xr_s);
    truck_s.x = []; truck_s.y = []; truck_s.theta = [];
    trailer_s.x = []; trailer_s.y = []; trailer_s.theta = [];
    target_xs_out = []; target_ys_out = [];
    
    if n_s_ref < 2
        fprintf('Simulate_vehicle: Reference path is too short to simulate.\n');
        truck_s.x = x0_truck_geom; truck_s.y = y0_truck_geom; truck_s.theta = th0_truck_geom_rad;
        trailer_s.x = x0_trailer_geom; trailer_s.y = y0_trailer_geom; trailer_s.theta = th0_trailer_geom_rad;
        if n_s_ref == 1, target_xs_out = xr_s(1); target_ys_out = yr_s(1); end
        return;
    end
    
    req_fields = {'WB_truck', 'look', 'v', 'dt', 'Ltr', 'Ltl', 'Lh'};
    for f = 1:length(req_fields)
        if ~isfield(g_geom, req_fields{f}) || isempty(g_geom.(req_fields{f}))
            error('g_geom.%s is missing or empty.', req_fields{f});
        end
    end
    if g_geom.v <= 0 || g_geom.dt <= 0, error('g_geom.v and g_geom.dt must be positive.'); end
    
    % --- Pre-allocate Simulation History Arrays ---
    path_dx = diff(xr_s);
    path_dy = diff(yr_s);
    est_path_len = sum(sqrt(path_dx.^2 + path_dy.^2));
    ideal_steps = ceil(est_path_len / (g_geom.v * g_geom.dt));
    max_sim_steps = ideal_steps * 2 + 100;
    max_sim_steps = max(max_sim_steps, n_s_ref + 20);
    max_sim_steps = min(max_sim_steps, 15000); % Safety cap
    
    truck_s_hist = struct('x', zeros(max_sim_steps, 1), 'y', zeros(max_sim_steps, 1), 'theta', zeros(max_sim_steps, 1));
    trailer_s_hist = struct('x', zeros(max_sim_steps, 1), 'y', zeros(max_sim_steps, 1), 'theta', zeros(max_sim_steps, 1));
    target_xs_hist = nan(max_sim_steps, 1);
    target_ys_hist = nan(max_sim_steps, 1);
    
    % Set initial conditions
    truck_s_hist.x(1) = x0_truck_geom;
    truck_s_hist.y(1) = y0_truck_geom;
    truck_s_hist.theta(1) = th0_truck_geom_rad;
    trailer_s_hist.x(1) = x0_trailer_geom;
    trailer_s_hist.y(1) = y0_trailer_geom;
    trailer_s_hist.theta(1) = th0_trailer_geom_rad;
    
    % --- Simulation Loop ---
    i_sim_step = 1;
    if nargin > 11 && strcmp(segment_type_str, 'turn_bezier')
        dist_thresh_stop = max(g_geom.v * g_geom.dt * 2.0, 0.25); % Looser tolerance for turns
    else
        dist_thresh_stop = max(g_geom.v * g_geom.dt * 1.5, 0.10); % Tighter for straight/PRM
    end
    
    min_dist_end_achieved = inf;
    stall_cnt = 0;
    max_stall_steps = ceil(g_geom.look / (g_geom.v * g_geom.dt)) + 20;
    dist_geom_center_to_front_bumper = g_geom.Ltr / 2;
    
    while i_sim_step < max_sim_steps
        curr_truck_x = truck_s_hist.x(i_sim_step);
        curr_truck_y = truck_s_hist.y(i_sim_step);
        curr_truck_th = truck_s_hist.theta(i_sim_step);
        
        % --- Stopping Condition ---
        dist_geom_to_ref_end = sqrt((xr_s(end) - curr_truck_x)^2 + (yr_s(end) - curr_truck_y)^2);
        if dist_geom_to_ref_end < dist_thresh_stop, break; end
        
        % --- Stall Detection (if stuck near the end) ---
        dist_sq_geom_path = (xr_s - curr_truck_x).^2 + (yr_s - curr_truck_y).^2;
        [~, idx_cl_geom] = min(dist_sq_geom_path);
        search_window = max(5, floor(g_geom.look / (est_path_len / n_s_ref + 1e-6)));
        if idx_cl_geom >= n_s_ref - search_window
            if dist_geom_to_ref_end < min_dist_end_achieved + 0.01
                stall_cnt = stall_cnt + 1;
            else
                min_dist_end_achieved = dist_geom_to_ref_end;
                stall_cnt = 0;
            end
            if stall_cnt > max_stall_steps, break; end
        else
            min_dist_end_achieved = inf;
            stall_cnt = 0;
        end
        
        % --- Pure Pursuit Controller ---
        % Find target point on path based on front bumper position
        curr_front_bumper_x = curr_truck_x + dist_geom_center_to_front_bumper * cos(curr_truck_th);
        curr_front_bumper_y = curr_truck_y + dist_geom_center_to_front_bumper * sin(curr_truck_th);
        
        dist_sq_fb_path = (xr_s - curr_front_bumper_x).^2 + (yr_s - curr_front_bumper_y).^2;
        [~, idx_cl_fb] = min(dist_sq_fb_path);
        
        % Search forward from the closest point to find the lookahead point
        cur_srch = idx_cl_fb;
        tgt_idx = idx_cl_fb;
        while cur_srch <= n_s_ref
            dist_fb_to_path_pt = sqrt((xr_s(cur_srch) - curr_front_bumper_x)^2 + (yr_s(cur_srch) - curr_front_bumper_y)^2);
            if dist_fb_to_path_pt >= g_geom.look
                tgt_idx = cur_srch;
                break;
            end
            if cur_srch == n_s_ref, tgt_idx = n_s_ref; break; end % Reached end of path
            cur_srch = cur_srch + 1;
        end
        xt = xr_s(tgt_idx); yt = yr_s(tgt_idx);
        target_xs_hist(i_sim_step) = xt; target_ys_hist(i_sim_step) = yt;
        
        % Calculate steering angle (delta)
        alpha = wrapToPi(atan2(yt - curr_front_bumper_y, xt - curr_front_bumper_x) - curr_truck_th);
        Le = max(0.1, sqrt((xt - curr_front_bumper_x)^2 + (yt - curr_front_bumper_y)^2));
        del = atan2(2 * g_geom.WB_truck * sin(alpha), Le);
        max_steer = deg2rad(40);
        del = max(-max_steer, min(max_steer, del));
        
        % --- Update Truck State ---
        truck_s_hist.x(i_sim_step + 1) = curr_truck_x + g_geom.v * cos(curr_truck_th) * g_geom.dt;
        truck_s_hist.y(i_sim_step + 1) = curr_truck_y + g_geom.v * sin(curr_truck_th) * g_geom.dt;
        truck_s_hist.theta(i_sim_step + 1) = wrapToPi(curr_truck_th + (g_geom.v / g_geom.WB_truck) * tan(del) * g_geom.dt);
        
        % --- Update Trailer State ---
        h_x = truck_s_hist.x(i_sim_step + 1) - (g_geom.Ltr / 2) * cos(truck_s_hist.theta(i_sim_step + 1));
        h_y = truck_s_hist.y(i_sim_step + 1) - (g_geom.Ltr / 2) * sin(truck_s_hist.theta(i_sim_step + 1));
        
        curr_tr_x = trailer_s_hist.x(i_sim_step);
        curr_tr_y = trailer_s_hist.y(i_sim_step);
        
        trailer_s_hist.theta(i_sim_step + 1) = wrapToPi(atan2(h_y - curr_tr_y, h_x - curr_tr_x));
        dist_h_to_tr_g = g_geom.Lh + (g_geom.Ltl / 2);
        trailer_s_hist.x(i_sim_step + 1) = h_x - dist_h_to_tr_g * cos(trailer_s_hist.theta(i_sim_step + 1));
        trailer_s_hist.y(i_sim_step + 1) = h_y - dist_h_to_tr_g * sin(trailer_s_hist.theta(i_sim_step + 1));
        
        i_sim_step = i_sim_step + 1;
    end
    
    if i_sim_step == max_sim_steps
        final_dist = sqrt((xr_s(end) - truck_s_hist.x(i_sim_step))^2 + (yr_s(end) - truck_s_hist.y(i_sim_step))^2);
        if final_dist >= dist_thresh_stop
            warning('Simulate_vehicle: Max sim steps (%d) reached. Final distance to reference end: %.2fm.', max_sim_steps, final_dist);
        end
    end
    
    % --- Trim and Output Results ---
    num_actual_steps = i_sim_step;
    truck_s.x = truck_s_hist.x(1:num_actual_steps);
    truck_s.y = truck_s_hist.y(1:num_actual_steps);
    truck_s.theta = truck_s_hist.theta(1:num_actual_steps);
    
    trailer_s.x = trailer_s_hist.x(1:num_actual_steps);
    trailer_s.y = trailer_s_hist.y(1:num_actual_steps);
    trailer_s.theta = trailer_s_hist.theta(1:num_actual_steps);
    
    if num_actual_steps > 0 && isnan(target_xs_hist(num_actual_steps)) && num_actual_steps > 1
        target_xs_hist(num_actual_steps) = target_xs_hist(num_actual_steps - 1);
        target_ys_hist(num_actual_steps) = target_ys_hist(num_actual_steps - 1);
    elseif num_actual_steps == 1
        target_xs_hist(1) = xr_s(1);
        target_ys_hist(1) = yr_s(1);
    elseif num_actual_steps == 0
        target_xs_out = []; target_ys_out = [];
        return;
    end
    
    target_xs_out = target_xs_hist(1:num_actual_steps);
    target_ys_out = target_ys_hist(1:num_actual_steps);
end

% -------------------------------------------------------------------------

function animate_vehicles_with_map(map, x_ref, y_ref, truck, trailer, target_xs, target_ys)
    % animate_vehicles_with_map Creates an animation of the vehicle simulation.
    
    if isempty(truck) || ~isfield(truck, 'x') || isempty(truck.x)
        fprintf('Animation skipped: Truck simulation data is empty.\n');
        return;
    end
    
    % Ensure target vectors are columns
    if ~isempty(target_xs) && isrow(target_xs), target_xs = target_xs'; end
    if ~isempty(target_ys) && isrow(target_ys), target_ys = target_ys'; end
    
    % --- Setup Figure and Axes ---
    fig_name_anim = 'Truck-Trailer on Open Field';
    existing_fig = findall(0, 'Type', 'figure', 'Name', fig_name_anim);
    
    if ~isempty(existing_fig)
        fig_anim = existing_fig(1);
        figure(fig_anim); % Bring to front
        ax_anim = fig_anim.CurrentAxes;
        cla(ax_anim); % Clear for new animation
    else
        fig_anim = figure('Name', fig_name_anim, 'Color', 'white', 'Position', [1000 50 800 700]);
        ax_anim = axes(fig_anim);
    end
    
    show(map, 'Parent', ax_anim);
    hold(ax_anim, 'on');
    axis(ax_anim, 'equal');
    grid(ax_anim, 'on');
    
    % --- Setup Plot Objects ---
    plot(ax_anim, x_ref, y_ref, 'k:', 'LineWidth', 1, 'DisplayName', 'Reference Path');
    truck_length_vis = 2.2; truck_width_vis = 1.0;
    trailer_length_vis = 1.0; trailer_width_vis = 1.0;
    
    truck_patch = patch(ax_anim, 'Vertices', zeros(4, 2), 'Faces', 1:4, 'FaceColor', 'blue', 'EdgeColor', 'k', 'FaceAlpha', 0.7, 'LineWidth', 1);
    trailer_patch = patch(ax_anim, 'Vertices', zeros(4, 2), 'Faces', 1:4, 'FaceColor', 'red', 'EdgeColor', 'k', 'FaceAlpha', 0.7, 'LineWidth', 1);
    hitch_line = plot(ax_anim, [0 0], [0 0], 'g-', 'LineWidth', 2, 'DisplayName', 'Hitch');
    target_marker_plot = plot(ax_anim, NaN, NaN, 'go', 'MarkerSize', 8, 'MarkerFaceColor', [0.2 0.8 0.2], 'LineWidth', 1.5, 'DisplayName', 'PP Target');
    
    legend(ax_anim, 'Location', 'eastoutside', 'AutoUpdate', 'off');
    
    % --- Animation Loop ---
    num_sim_steps = length(truck.x);
    if num_sim_steps == 0, return; end
    
    target_frames_for_anim = 150; % Aim for this many frames
    anim_step = max(1, floor(num_sim_steps / target_frames_for_anim));
    
    for i = 1:anim_step:num_sim_steps
        % Update Pure Pursuit target marker
        if ~isempty(target_xs) && i <= length(target_xs) && ~isnan(target_xs(i))
            set(target_marker_plot, 'XData', target_xs(i), 'YData', target_ys(i));
        else
            set(target_marker_plot, 'XData', NaN, 'YData', NaN);
        end
        
        % Update truck patch
        set(truck_patch, 'Vertices', get_rectangle_for_anim(truck.x(i), truck.y(i), truck.theta(i), truck_length_vis, truck_width_vis));
        
        % Update trailer patch and hitch
        if isfield(trailer, 'x') && ~isempty(trailer.x) && i <= length(trailer.x) && ~isnan(trailer.x(i))
            set(trailer_patch, 'Vertices', get_rectangle_for_anim(trailer.x(i), trailer.y(i), trailer.theta(i), trailer_length_vis, trailer_width_vis));
            
            h_trk_r_x = truck.x(i) - (truck_length_vis / 2) * cos(truck.theta(i));
            h_trk_r_y = truck.y(i) - (truck_length_vis / 2) * sin(truck.theta(i));
            tr_frnt_x = trailer.x(i) + (trailer_length_vis / 2) * cos(trailer.theta(i));
            tr_frnt_y = trailer.y(i) + (trailer_length_vis / 2) * sin(trailer.theta(i));
            set(hitch_line, 'XData', [h_trk_r_x tr_frnt_x], 'YData', [h_trk_r_y tr_frnt_y]);
        else
            set(trailer_patch, 'Vertices', nan(4, 2));
            set(hitch_line, 'XData', [NaN NaN], 'YData', [NaN NaN]);
        end
        
        drawnow limitrate;
        pause(0.005);
    end
    
    % --- Draw Final Pose ---
    % Ensures the very last step is shown, even if skipped by the animation step
    if ~isempty(truck.x) && ~isnan(truck.x(end))
        final_idx = length(truck.x);
        
        if ~isempty(target_xs) && final_idx <= length(target_xs) && ~isnan(target_xs(final_idx))
            set(target_marker_plot, 'XData', target_xs(final_idx), 'YData', target_ys(final_idx));
        else
            set(target_marker_plot, 'XData', NaN, 'YData', NaN);
        end
        
        set(truck_patch, 'Vertices', get_rectangle_for_anim(truck.x(final_idx), truck.y(final_idx), truck.theta(final_idx), truck_length_vis, truck_width_vis));
        
        if isfield(trailer, 'x') && ~isempty(trailer.x) && final_idx <= length(trailer.x) && ~isnan(trailer.x(final_idx))
            set(trailer_patch, 'Vertices', get_rectangle_for_anim(trailer.x(final_idx), trailer.y(final_idx), trailer.theta(final_idx), trailer_length_vis, trailer_width_vis));
            h_trk_r_x = truck.x(final_idx) - (truck_length_vis / 2) * cos(truck.theta(final_idx));
            h_trk_r_y = truck.y(final_idx) - (truck_length_vis / 2) * sin(truck.theta(final_idx));
            tr_frnt_x = trailer.x(final_idx) + (trailer_length_vis / 2) * cos(trailer.theta(final_idx));
            tr_frnt_y = trailer.y(final_idx) + (trailer_length_vis / 2) * sin(trailer.theta(final_idx));
            set(hitch_line, 'XData', [h_trk_r_x tr_frnt_x], 'YData', [h_trk_r_y tr_frnt_y]);
        else
            set(trailer_patch, 'Vertices', nan(4, 2));
            set(hitch_line, 'XData', [NaN NaN], 'YData', [NaN NaN]);
        end
        drawnow;
    end
end

% -------------------------------------------------------------------------

function rect = get_rectangle_for_anim(x_center, y_center, theta_rad, L, W)
    % get_rectangle_for_anim Calculates the 4 corner vertices of a rotated rectangle.
    
    R_matrix = [cos(theta_rad) -sin(theta_rad); sin(theta_rad) cos(theta_rad)];
    
    % Corners in local frame (centered at origin)
    corners_local = [-L/2 -W/2; L/2 -W/2; L/2 W/2; -L/2 W/2];
    
    % Rotate and translate corners to world frame
    rect = (R_matrix * corners_local')' + repmat([x_center y_center], 4, 1);
end