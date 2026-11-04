% vineyardNavigator_SegmentBySegment.m
% Main script for step-by-step path planning and execution in a vineyard,
% using an image-based map and defined snap points for navigation tasks.

clear; clc;
% close all; % Comment/uncomment as needed for figure persistence

fprintf('--- Vineyard Navigation System (Segment-by-Segment) ---\n');

% --- 1. Load Vineyard Parameters & Initialize Map ---
VINEYARD_PARAMS = defineVineyardParameters(); 
g_config.map_resolution = 10; 
fprintf('Step 0: Generating Occupancy Map from filari.png...\n');
try, topView = imread('filari.png'); catch ME, error('filari.png not found. %s', ME.message); end
if size(topView,3) == 3, topView = rgb2gray(topView); end
imageMask = imbinarize(topView); 
imageMask = ~imageMask; % IMPORTANT: Assumes filari.png: BLACK obstacles on WHITE background.
                        % If WHITE obstacles on BLACK background, COMMENT OUT THIS LINE.
imageMaskResized = imresize(imageMask, [50*g_config.map_resolution, 50*g_config.map_resolution]);
environmentMap = binaryOccupancyMap(50, 50, g_config.map_resolution);
setOccupancy(environmentMap, imageMaskResized); 
UPDATED_VINEYARD_PARAMS = VINEYARD_PARAMS;
UPDATED_VINEYARD_PARAMS.map_origin_offset = [environmentMap.XWorldLimits(1), environmentMap.YWorldLimits(1)];
UPDATED_VINEYARD_PARAMS.row_y_start_param_ref = VINEYARD_PARAMS.row_y_start; 
UPDATED_VINEYARD_PARAMS.row_y_end_param_ref   = VINEYARD_PARAMS.row_y_end;

fig_main_handle = figure('Name', 'Vineyard Task Environment', 'Position', [50 50 900 750]);
ax_main = axes(fig_main_handle);
show(environmentMap, 'Parent', ax_main); hold(ax_main, 'on');
title(ax_main, 'Vineyard Map & Task Waypoints'); axis(ax_main, 'equal'); grid(ax_main, 'on');

% --- 2. Define Snap Points & Task Sequence (Extended U-Turn Example) ---
all_snap_points = collectAllSnapPoints(UPDATED_VINEYARD_PARAMS);
task_sequence_names = { ...
    'Row_1_BottomEntry', ... 
    'Row_1_TopExit', ...    
    'Row_2_TopEntry', ...  % Enters Row 2 from Top, travels "down"
    'Row_2_BottomExit', ... % Exits Row 2 at Bottom
    'Row_3_BottomEntry', ... % << NEW: Turn in Bottom Headland, enter Row 3 from Bottom (travels "up")
    'Row_3_TopExit',...
    'Row_1_TopEntry',...
    'Row_1_BottomExit'
   
};
if length(task_sequence_names) < 2, error('Task sequence too short.'); end
task_waypoints_full_sequence = getWaypointsFromTaskSequence(task_sequence_names, all_snap_points);
for i=1:length(task_waypoints_full_sequence)
    plot(ax_main, task_waypoints_full_sequence(i).xy(1), task_waypoints_full_sequence(i).xy(2), 'yd', 'MarkerFaceColor', 'y', 'MarkerSize', 7, 'DisplayName', 'Task Waypoint');
    text(ax_main, task_waypoints_full_sequence(i).xy(1)+0.3, task_waypoints_full_sequence(i).xy(2), strrep(task_waypoints_full_sequence(i).name, '_', ' '), 'Color', 'blue', 'FontSize', 8);
end
legend(ax_main, 'off'); legend(ax_main, 'show', 'Location', 'southoutside');

% --- 3. Initialize Vehicle State & Full Path Accumulation Variables ---
current_vehicle_pose_world = [task_waypoints_full_sequence(1).xy, task_waypoints_full_sequence(1).theta_rad]; 
prev_trailer_sim_end_theta = current_vehicle_pose_world(3); 
x_path_full_executed_ref = []; y_path_full_executed_ref = [];

% --- 4. Centralized Vehicle Definitions & Costmaps ---
current_geom = geom(); 
current_geom.dt = 0.05; 
current_geom.v = 0.5; % Speed for simulation (used by simulate_vehicle_from_pose)
current_geom.look = 0.7; % Lookahead for simulation (used by simulate_vehicle_from_pose) - TUNE THIS
current_geom.WB_truck = current_geom.Ltr * 0.70; % IMPORTANT for simulate_vehicle_from_pose
current_geom.WB_trailer = current_geom.Ltl * 0.70; % For consistency if needed

RO_truck_param = current_geom.Ltr * 0.15; WB_truck_param_for_dims = current_geom.WB_truck; 
FO_truck_param = current_geom.Ltr - WB_truck_param_for_dims - RO_truck_param; if FO_truck_param < -1e-6, FO_truck_param = max(0, FO_truck_param); end
truck_dimensions = vehicleDimensions(current_geom.Ltr, current_geom.Wtr, current_geom.Wtr, 'Wheelbase', WB_truck_param_for_dims, 'RearOverhang', RO_truck_param, 'FrontOverhang', FO_truck_param);
truck_checker_hard = inflationCollisionChecker(truck_dimensions, 'NumCircles', 5);
RO_trailer_param = current_geom.Ltl * 0.15; WB_trailer_param_for_dims = current_geom.WB_trailer;
FO_trailer_param = current_geom.Ltl - WB_trailer_param_for_dims - RO_trailer_param; if FO_trailer_param < -1e-6, FO_trailer_param = max(0, FO_trailer_param); end
trailer_dimensions = vehicleDimensions(current_geom.Ltl, current_geom.Wtl, current_geom.Wtl, 'Wheelbase', WB_trailer_param_for_dims, 'RearOverhang', RO_trailer_param, 'FrontOverhang', FO_trailer_param);
trailer_checker_hard = inflationCollisionChecker(trailer_dimensions, 'NumCircles', 3);

costmap_truck_hard = vehicleCostmap(environmentMap, 'CollisionChecker', truck_checker_hard);
costmap_trailer_hard = vehicleCostmap(environmentMap, 'CollisionChecker', trailer_checker_hard);
CLOSE_CALL_THRESHOLD_VINEYARD = 0.15; 
costmap_truck_close = []; costmap_trailer_close = [];
if CLOSE_CALL_THRESHOLD_VINEYARD > 0
    inflated_map_for_close_calls = copy(environmentMap); inflate(inflated_map_for_close_calls, CLOSE_CALL_THRESHOLD_VINEYARD);
    costmap_truck_close = vehicleCostmap(inflated_map_for_close_calls, 'CollisionChecker', truck_checker_hard);
    costmap_trailer_close = vehicleCostmap(inflated_map_for_close_calls, 'CollisionChecker', trailer_checker_hard);
end
app_costmaps = struct('truck_hard',costmap_truck_hard,'trailer_hard',costmap_trailer_hard,'truck_close',costmap_truck_close,'trailer_close',costmap_trailer_close);
fprintf('Vehicle definitions and costmaps created.\n');

% --- 5. Loop Through Task Segments ---
h_plots_on_main_ax = gobjects(0); 

for seg_idx = 1:(length(task_waypoints_full_sequence)-1)
    wp_start_details = task_waypoints_full_sequence(seg_idx);     
    wp_end_details = task_waypoints_full_sequence(seg_idx+1);       
    fprintf('\n--- Segment %d: From %s to %s ---\n', seg_idx, wp_start_details.name, wp_end_details.name);
    fprintf('Vehicle current start: X=%.2f, Y=%.2f, Th=%.2f rad. Prev Trailer Th=%.2f\n', current_vehicle_pose_world(1), current_vehicle_pose_world(2), current_vehicle_pose_world(3), prev_trailer_sim_end_theta);

    if ~isempty(h_plots_on_main_ax)
        valid_handles_to_delete = h_plots_on_main_ax(isgraphics(h_plots_on_main_ax)); 
        if ~isempty(valid_handles_to_delete), delete(valid_handles_to_delete); end
    end
    h_plots_on_main_ax = gobjects(0); 

    actual_vehicle_start_pose_for_segment_xy_th = current_vehicle_pose_world; 
    [x_seg_ref, y_seg_ref, theta_seg_ref, segment_type, h_bezier_debug_plots_on_main] = ...
        generateSingleSegmentPath(wp_start_details, actual_vehicle_start_pose_for_segment_xy_th, wp_end_details, UPDATED_VINEYARD_PARAMS, ax_main); 
    h_plots_on_main_ax = [h_plots_on_main_ax, h_bezier_debug_plots_on_main(isgraphics(h_bezier_debug_plots_on_main))]; % Add valid handles

    if isempty(x_seg_ref) || length(x_seg_ref) < 2, fprintf('Seg %d path gen failed/short. Skipping.\n', seg_idx); %input('Error. Enter...'); continue; 
    end
    
    fprintf('INFO: For Segment %d (%s):\n', seg_idx, segment_type);
    fprintf('  Actual vehicle start: X=%.2f, Y=%.2f, Theta=%.2f rad\n', actual_vehicle_start_pose_for_segment_xy_th);
    fprintf('  Generated Ref Path (x_seg_ref) has %d points.\n', length(x_seg_ref));
    if ~isempty(x_seg_ref)
        fprintf('  Ref Path Start: X=%.2f, Y=%.2f, Theta=%.2f rad\n', x_seg_ref(1), y_seg_ref(1), theta_seg_ref(1));
        fprintf('  Ref Path End:   X=%.2f, Y=%.2f, Theta=%.2f rad\n', x_seg_ref(end), y_seg_ref(end), theta_seg_ref(end));
    end
    fprintf('  Target Snap Pt: X=%.2f, Y=%.2f, Theta=%.2f rad (wp_end_details)\n', wp_end_details.xy(1), wp_end_details.xy(2), wp_end_details.theta_rad);

    h_current_segment_plot = plot(ax_main, x_seg_ref, y_seg_ref, 'c--', 'LineWidth', 1.5, 'DisplayName', 'Current Segment (Initial Ref)');
    h_plots_on_main_ax(end+1) = h_current_segment_plot; 
    legend(ax_main, 'off'); legend(ax_main, 'show', 'Location', 'southoutside'); 
   % input(sprintf('Segment %d ref path generated. Press Enter to simulate...', seg_idx), 's');
    
    fprintf('Simulating initial reference path for segment %d...\n', seg_idx);
    x0_t = current_vehicle_pose_world(1); y0_t = current_vehicle_pose_world(2); th0_t_rad = current_vehicle_pose_world(3);
    th0_tr_rad = prev_trailer_sim_end_theta;
    hitch_x_curr = x0_t - (current_geom.Ltr/2) * cos(th0_t_rad); hitch_y_curr = y0_t - (current_geom.Ltr/2) * sin(th0_t_rad);
    dist_h_to_tr_geom = current_geom.Lh + (current_geom.Ltl / 2);
    x0_tr = hitch_x_curr - dist_h_to_tr_geom * cos(th0_tr_rad); y0_tr = hitch_y_curr - dist_h_to_tr_geom * sin(th0_tr_rad);

    [truck_sim_seg_orig, trailer_sim_seg_orig, target_xs_orig_seg, target_ys_orig_seg] = ...
        simulate_vehicle_from_pose(x_seg_ref,y_seg_ref,x0_t,y0_t,th0_t_rad,x0_tr,y0_tr,th0_tr_rad,current_geom,RO_truck_param,RO_trailer_param, segment_type);

    if ~isempty(truck_sim_seg_orig.x)
        animate_vehicles_with_map(environmentMap, x_seg_ref, y_seg_ref, truck_sim_seg_orig, trailer_sim_seg_orig, target_xs_orig_seg, target_ys_orig_seg);
        anim_fig_handles = findall(0,'Type','figure','-regexp','Name','Truck-Trailer on Open Field'); 
        if ~isempty(anim_fig_handles), title(anim_fig_handles(1).CurrentAxes, sprintf('Anim: Seg %d (Initial): %s to %s', seg_idx, strrep(wp_start_details.name,'_',' '), strrep(wp_end_details.name,'_',' '))); end
    else, fprintf('Skipping animation: initial sim seg %d empty.\n', seg_idx); end
   % input(sprintf('Seg %d initial animation complete. Press Enter...', seg_idx), 's');

    x_seg_mod = x_seg_ref; y_seg_mod = y_seg_ref; theta_seg_mod = theta_seg_ref; 
    truck_sim_seg_final = truck_sim_seg_orig; trailer_sim_seg_final = trailer_sim_seg_orig;
    target_xs_sim_final = target_xs_orig_seg; target_ys_sim_final = target_ys_orig_seg; % For animation

    RUN_MODIFY_CURVES = false; 
    if strcmp(segment_type, 'turn_bezier') && ~isempty(truck_sim_seg_orig.x) && RUN_MODIFY_CURVES
        fprintf('Seg %d headland turn. Applying Recker-Bezier...\n', seg_idx);
        global_hotspots_for_segment = []; curve_bounds_single_seg = [1, length(x_seg_ref), 0]; 
        current_geom_for_modify = current_geom; 
        [x_seg_mod_temp, y_seg_mod_temp, overall_modification_successful_seg] = ...
            modify_curves_bezier(x_seg_ref, y_seg_ref, theta_seg_ref, curve_bounds_single_seg, ...
            environmentMap, truck_sim_seg_orig, trailer_sim_seg_orig, current_geom_for_modify, RO_truck_param, RO_trailer_param, ...
            app_costmaps, truck_dimensions, trailer_dimensions, global_hotspots_for_segment);
        if overall_modification_successful_seg && ~isempty(x_seg_mod_temp) && length(x_seg_mod_temp) >= 2
            x_seg_mod = x_seg_mod_temp; y_seg_mod = y_seg_mod_temp;
            dx_mod=diff(x_seg_mod); dy_mod=diff(y_seg_mod); theta_calc_mod=atan2(dy_mod,dx_mod); theta_seg_mod=[theta_calc_mod(:)',theta_calc_mod(end)];             
            fprintf('Seg %d Recker-Bezier mod OK.\n', seg_idx);
            h_mod_plot = plot(ax_main, x_seg_mod, y_seg_mod, 'g-', 'LineWidth',2,'DisplayName','Segment (Modified)'); h_plots_on_main_ax(end+1)=h_mod_plot; legend(ax_main,'off'); legend(ax_main,'show');
            fprintf('Simulating MODIFIED path for segment %d...\n', seg_idx);
            x0_t_mod = current_vehicle_pose_world(1); y0_t_mod = current_vehicle_pose_world(2); th0_t_rad_mod = current_vehicle_pose_world(3);
            th0_tr_rad_mod = prev_trailer_sim_end_theta; 
            hitch_x_mod = x0_t_mod - (current_geom.Ltr/2)*cos(th0_t_rad_mod); hitch_y_mod = y0_t_mod - (current_geom.Ltr/2)*sin(th0_t_rad_mod);
            x0_tr_mod = hitch_x_mod - dist_h_to_tr_geom * cos(th0_tr_rad_mod); y0_tr_mod = hitch_y_mod - dist_h_to_tr_geom * sin(th0_tr_rad_mod);
            [truck_sim_seg_final, trailer_sim_seg_final, target_xs_sim_final, target_ys_sim_final] = simulate_vehicle_from_pose(x_seg_mod,y_seg_mod,x0_t_mod,y0_t_mod,th0_t_rad_mod,x0_tr_mod,y0_tr_mod,th0_tr_rad_mod,current_geom,RO_truck_param,RO_trailer_param, segment_type); 
        else
            fprintf('!!! Seg %d Recker-Bezier mod FAILED. Using initial. !!!\n', seg_idx);
        end
        if ~isempty(truck_sim_seg_final.x)
            animate_vehicles_with_map(environmentMap, x_seg_mod, y_seg_mod, truck_sim_seg_final, trailer_sim_seg_final, target_xs_sim_final, target_ys_sim_final);
            anim_fig_handles = findall(0,'Type','figure','-regexp','Name','Truck-Trailer on Open Field');
            if ~isempty(anim_fig_handles), title(anim_fig_handles(1).CurrentAxes, sprintf('Anim: Seg %d (Final): %s to %s', seg_idx, strrep(wp_start_details.name,'_',' '), strrep(wp_end_details.name,'_',' '))); end
        else, fprintf('Skipping anim: final sim seg %d empty.\n', seg_idx); end
       % input(sprintf('Seg %d final anim complete. Press Enter...', seg_idx), 's');
    elseif strcmp(segment_type, 'turn_bezier') && ~RUN_MODIFY_CURVES
         fprintf('Skipped Recker-Bezier for headland turn segment %d.\n', seg_idx);
    end

    final_executed_ref_x_seg = x_seg_mod; final_executed_ref_y_seg = y_seg_mod;
    if isempty(x_path_full_executed_ref), x_path_full_executed_ref = final_executed_ref_x_seg; y_path_full_executed_ref = final_executed_ref_y_seg;
    else, x_path_full_executed_ref = [x_path_full_executed_ref, final_executed_ref_x_seg(2:end)]; y_path_full_executed_ref = [y_path_full_executed_ref, final_executed_ref_y_seg(2:end)]; end
    
    last_valid_truck_sim = []; last_valid_trailer_sim = [];
    if ~isempty(truck_sim_seg_final) && ~isempty(truck_sim_seg_final.x), last_valid_truck_sim = truck_sim_seg_final; last_valid_trailer_sim = trailer_sim_seg_final;
    elseif ~isempty(truck_sim_seg_orig) && ~isempty(truck_sim_seg_orig.x), last_valid_truck_sim = truck_sim_seg_orig; last_valid_trailer_sim = trailer_sim_seg_orig; end

    if ~isempty(last_valid_truck_sim) && isfield(last_valid_truck_sim, 'x') && ~isempty(last_valid_truck_sim.x) && ~isempty(last_valid_truck_sim.theta)
        current_vehicle_pose_world = [last_valid_truck_sim.x(end), last_valid_truck_sim.y(end), last_valid_truck_sim.theta(end)];
        if ~isempty(last_valid_trailer_sim) && isfield(last_valid_trailer_sim,'theta') && ~isempty(last_valid_trailer_sim.theta) 
            prev_trailer_sim_end_theta = last_valid_trailer_sim.theta(end);
        else, prev_trailer_sim_end_theta = current_vehicle_pose_world(3); fprintf('Warn: Trailer sim theta empty seg %d end.\n', seg_idx); end
    else
        fprintf('CRITICAL WARN: No valid sim data for seg %d. Using geometric end of ref path.\n', seg_idx);
        if ~isempty(theta_seg_mod), current_vehicle_pose_world = [final_executed_ref_x_seg(end), final_executed_ref_y_seg(end), theta_seg_mod(end)]; prev_trailer_sim_end_theta = theta_seg_mod(end); 
        else, current_vehicle_pose_world = [final_executed_ref_x_seg(end), final_executed_ref_y_seg(end), current_vehicle_pose_world(3)]; prev_trailer_sim_end_theta = current_vehicle_pose_world(3); end
    end
    fprintf('Vehicle pose after seg %d: X=%.2f,Y=%.2f,Th=%.2f rad. PrevTrailerTh=%.2f\n', seg_idx, current_vehicle_pose_world(1), current_vehicle_pose_world(2), current_vehicle_pose_world(3), prev_trailer_sim_end_theta);
end
fprintf('\n--- All Segments Processed ---\n');
figure(fig_main_handle);
plot(ax_main, x_path_full_executed_ref, y_path_full_executed_ref, 'm.-', 'LineWidth', 1.2, 'DisplayName', 'Full Planned Path (Concatenated Ref Segments)');
legend(ax_main, 'off'); legend(ax_main, 'show', 'Location', 'southoutside'); 
title(ax_main, 'Full Path Execution Plan (Concatenated Reference Segments)');

% ====== HELPER FUNCTIONS ======
% --- defineVineyardParameters ---
function VINEYARD_PARAMS = defineVineyardParameters()
    VINEYARD_PARAMS.field_x_limits = [1, 49]; 
    VINEYARD_PARAMS.bottom_headland_y_limits = [2.3, 6]; 
    VINEYARD_PARAMS.top_headland_y_limits = [44.5, 48];    
    VINEYARD_PARAMS.row_y_start = VINEYARD_PARAMS.bottom_headland_y_limits(2)+0.5; 
    VINEYARD_PARAMS.row_y_end = VINEYARD_PARAMS.top_headland_y_limits(1)-0.5;     
    VINEYARD_PARAMS.row_y_bottom_snap = VINEYARD_PARAMS.bottom_headland_y_limits(2) + 0.5; 
    VINEYARD_PARAMS.row_y_top_snap    = VINEYARD_PARAMS.top_headland_y_limits(1) - 0.5;
    row_centers_x = [3, 7, 10.75]; 
    VINEYARD_PARAMS.rows = [];
    for i = 1:length(row_centers_x)
        VINEYARD_PARAMS.rows(i).name = sprintf('Row_%d', i); VINEYARD_PARAMS.rows(i).center_x = row_centers_x(i);
        VINEYARD_PARAMS.rows(i).bottom_entry_xy = [row_centers_x(i), VINEYARD_PARAMS.row_y_bottom_snap]; VINEYARD_PARAMS.rows(i).bottom_entry_theta_rad = pi/2; 
        VINEYARD_PARAMS.rows(i).top_exit_xy = [row_centers_x(i), VINEYARD_PARAMS.row_y_top_snap]; VINEYARD_PARAMS.rows(i).top_exit_theta_rad = pi/2;  
        VINEYARD_PARAMS.rows(i).top_entry_xy = [row_centers_x(i), VINEYARD_PARAMS.row_y_top_snap]; VINEYARD_PARAMS.rows(i).top_entry_theta_rad = -pi/2; 
        VINEYARD_PARAMS.rows(i).bottom_exit_xy = [row_centers_x(i), VINEYARD_PARAMS.row_y_bottom_snap]; VINEYARD_PARAMS.rows(i).bottom_exit_theta_rad = -pi/2;
    end
    if length(VINEYARD_PARAMS.rows) >= 2, VINEYARD_PARAMS.row_width_center_to_center = VINEYARD_PARAMS.rows(2).center_x - VINEYARD_PARAMS.rows(1).center_x; else, VINEYARD_PARAMS.row_width_center_to_center = 4; end
    VINEYARD_PARAMS.parking_spots = [];
    VINEYARD_PARAMS.parking_spots(1).name = 'Parking_P1'; VINEYARD_PARAMS.parking_spots(1).coords_xy = [4, 4]; VINEYARD_PARAMS.parking_spots(1).theta_rad = deg2rad(0);  
end

% --- collectAllSnapPoints ---
function all_snap_points = collectAllSnapPoints(VINEYARD_PARAMS)
    all_snap_points = struct('name', {}, 'xy', {}, 'theta_rad', {});
    for i = 1:length(VINEYARD_PARAMS.rows)
        all_snap_points(end+1) = struct('name', [VINEYARD_PARAMS.rows(i).name '_BottomEntry'], 'xy', VINEYARD_PARAMS.rows(i).bottom_entry_xy, 'theta_rad', VINEYARD_PARAMS.rows(i).bottom_entry_theta_rad);
        all_snap_points(end+1) = struct('name', [VINEYARD_PARAMS.rows(i).name '_TopExit'],    'xy', VINEYARD_PARAMS.rows(i).top_exit_xy,    'theta_rad', VINEYARD_PARAMS.rows(i).top_exit_theta_rad);
        all_snap_points(end+1) = struct('name', [VINEYARD_PARAMS.rows(i).name '_TopEntry'],    'xy', VINEYARD_PARAMS.rows(i).top_entry_xy,    'theta_rad', VINEYARD_PARAMS.rows(i).top_entry_theta_rad);
        all_snap_points(end+1) = struct('name', [VINEYARD_PARAMS.rows(i).name '_BottomExit'],  'xy', VINEYARD_PARAMS.rows(i).bottom_exit_xy,  'theta_rad', VINEYARD_PARAMS.rows(i).bottom_exit_theta_rad);
    end
    if isfield(VINEYARD_PARAMS, 'parking_spots') && ~isempty(VINEYARD_PARAMS.parking_spots)
        for i = 1:length(VINEYARD_PARAMS.parking_spots), all_snap_points(end+1) = struct('name', VINEYARD_PARAMS.parking_spots(i).name, 'xy', VINEYARD_PARAMS.parking_spots(i).coords_xy, 'theta_rad', VINEYARD_PARAMS.parking_spots(i).theta_rad); end
    end
end

% --- getWaypointsFromTaskSequence ---
function waypoints = getWaypointsFromTaskSequence(task_sequence_names, all_snap_points)
    waypoints = struct('name', {}, 'xy', {}, 'theta_rad', {}, 'type', {}, 'theta_rad_implicit_exit', {});
    if isempty(task_sequence_names), warning('Task sequence is empty.'); return; end
    for i = 1:length(task_sequence_names)
        idx = find(strcmp({all_snap_points.name}, task_sequence_names{i}));
        if isempty(idx), error('Snap point name "%s" not found.', task_sequence_names{i}); end
        waypoints(i).name=all_snap_points(idx).name; waypoints(i).xy=all_snap_points(idx).xy; waypoints(i).theta_rad=all_snap_points(idx).theta_rad;
        if contains(waypoints(i).name,'Parking'),waypoints(i).type='parking';elseif contains(waypoints(i).name,'Entry'),waypoints(i).type='row_entry';elseif contains(waypoints(i).name,'Exit'),waypoints(i).type='row_exit';else,waypoints(i).type='unknown';end
    end
    for i=1:length(waypoints)-1,p_start=waypoints(i).xy;p_end=waypoints(i+1).xy;waypoints(i).theta_rad_implicit_exit=atan2(p_end(2)-p_start(2),p_end(1)-p_start(1));end
    if ~isempty(waypoints),waypoints(end).theta_rad_implicit_exit=waypoints(end).theta_rad;end
end

% --- geom (Geometric parameters of the vehicle) ---
function g = geom()
    g.Ltr = 2.20; % Overall length of truck (geometric center to front/rear bumper)
    g.Wtr = 1.00; % Width of truck
    g.Ltl = 1.0;  % Overall length of trailer (Your simulate_vehicle_from_pose uses g_geom.Ltl / 2 for trailer center)
    g.Wtl = 1.00; % Width of trailer
    g.Lh  = 1.00; % Hitch length from truck REAR AXLE to trailer FRONT AXLE (pivot point on trailer)
                  % This definition is used in your simulate_vehicle_from_pose's trailer update:
                  % dist_hitch_to_trailer_geom_center = g_geom.Lh + (g_geom.Ltl / 2);
                  % If Lh is from truck REAR AXLE, then truck_s.x(i+1) - (g_geom.Ltr/2)*cos... is correct for hitch_x
end

% --- generateSingleSegmentPath (calls generateInitialHeadlandBezier_V2) ---
function [x_seg, y_seg, theta_seg, segment_type_str, h_bezier_debug_plots_on_main] = generateSingleSegmentPath(...
    wp_start_snap_details_struct, actual_vehicle_start_pose_xy_theta, ...
    wp_end_snap_details_struct, VINEYARD_PARAMS, ax_for_debug_plot)

    points_per_meter_straight = 20; 
    points_per_meter_turn = 20;     
    P0_actual = actual_vehicle_start_pose_xy_theta(1:2);
    Theta0_actual = actual_vehicle_start_pose_xy_theta(3);
    P3_target = wp_end_snap_details_struct.xy;
    Theta3_target_entry_orientation = wp_end_snap_details_struct.theta_rad;
    segment_type_str = 'straight_connecting'; 
    h_bezier_debug_plots_on_main = gobjects(0); 
    is_in_row = false;
    if (strcmp(wp_start_snap_details_struct.type, 'row_entry') && strcmp(wp_end_snap_details_struct.type, 'row_exit')) || ...
       (strcmp(wp_start_snap_details_struct.type, 'row_exit') && strcmp(wp_end_snap_details_struct.type, 'row_entry') && abs(P0_actual(1) - P3_target(1)) < 0.1 ) 
        start_parts = strsplit(wp_start_snap_details_struct.name, '_'); end_parts = strsplit(wp_end_snap_details_struct.name, '_');
        if strcmp(start_parts{2}, end_parts{2}), is_in_row = true; segment_type_str = 'in_row'; end % Compare Row_N part
    end
    is_headland_turn = false;
    if (strcmp(wp_start_snap_details_struct.type, 'row_exit') && strcmp(wp_end_snap_details_struct.type, 'row_entry'))
        start_row_parts = strsplit(wp_start_snap_details_struct.name, '_'); end_row_parts = strsplit(wp_end_snap_details_struct.name, '_');
        if ~strcmp(start_row_parts{2}, end_row_parts{2}), is_headland_turn = true; segment_type_str = 'turn_bezier'; end
    end
    x_seg = []; y_seg = []; 
    if is_in_row || (~is_in_row && ~is_headland_turn) 
        dist_segment = norm(P3_target - P0_actual); num_pts = ceil(dist_segment * points_per_meter_straight); num_pts = max(2, num_pts); 
        x_seg = linspace(P0_actual(1), P3_target(1), num_pts); y_seg = linspace(P0_actual(2), P3_target(2), num_pts);
        fprintf('  Generated STRAIGHT path of %d points, length approx %.2fm.\n', num_pts, dist_segment);
    elseif is_headland_turn 
        if P0_actual(2) > (VINEYARD_PARAMS.bottom_headland_y_limits(2) + VINEYARD_PARAMS.top_headland_y_limits(1))/2, headland_y_limits = VINEYARD_PARAMS.top_headland_y_limits; else, headland_y_limits = VINEYARD_PARAMS.bottom_headland_y_limits; end
        headland_depth = abs(headland_y_limits(2) - headland_y_limits(1));
        [x_seg, y_seg, ~, h_bezier_debug_plots_on_main] = generateInitialHeadlandBezier_V2(P0_actual, Theta0_actual, P3_target, Theta3_target_entry_orientation, VINEYARD_PARAMS, headland_depth, points_per_meter_turn, ax_for_debug_plot);
    else 
        fprintf('WARNING: Segment type unclear for path from %s to %s. Generating a simple straight line.\n', wp_start_snap_details_struct.name, wp_end_snap_details_struct.name);
        dist_segment = norm(P3_target - P0_actual); num_pts = ceil(dist_segment * points_per_meter_straight); num_pts = max(2,num_pts);
        x_seg = linspace(P0_actual(1), P3_target(1), num_pts); y_seg = linspace(P0_actual(2), P3_target(2), num_pts);
    end
    if ~isempty(x_seg) && length(x_seg) > 1, dx=diff(x_seg); dy=diff(y_seg); theta_seg_calc=atan2(dy,dx); theta_seg=[theta_seg_calc(:)', theta_seg_calc(end)]; theta_seg=unwrap(theta_seg);
    elseif ~isempty(x_seg) && length(x_seg)==1, theta_seg = Theta0_actual; 
    else, theta_seg=[]; x_seg=[]; y_seg=[]; fprintf('Warning: Empty path segment generated for task: %s to %s.\n', wp_start_snap_details_struct.name, wp_end_snap_details_struct.name); end
end

% --- generateInitialHeadlandBezier_V2 ---
function [x_bezier, y_bezier, control_points_struct, h_debug_plots] = generateInitialHeadlandBezier_V2(...
    P0_actual, theta0_actual, P3_target, theta3_target_entry, VINEYARD_PARAMS, ...
    headland_depth, points_per_meter, ax_to_plot_on)
    h_debug_plots = gobjects(0);
    d_factor_longitudinal_P1 = 1.0; d_factor_lateral_P1    = 1.0; 
    d_factor_longitudinal_P2 = 1.0; d_factor_lateral_P2    = 1.0;
    base_dist_P1 = max(VINEYARD_PARAMS.row_width_center_to_center * 0.8, headland_depth * 0.5); base_dist_P2 = base_dist_P1;
    d1_long = base_dist_P1 * d_factor_longitudinal_P1; d1_lat  = VINEYARD_PARAMS.row_width_center_to_center * d_factor_lateral_P1;
    d2_long = base_dist_P2 * d_factor_longitudinal_P2; d2_lat  = VINEYARD_PARAMS.row_width_center_to_center * d_factor_lateral_P2;
    dir_P0 = [cos(theta0_actual), sin(theta0_actual)];
    delta_x_turn = P3_target(1) - P0_actual(1);
    if abs(wrapToPi(theta0_actual - pi/2)) < pi/4, turn_direction_k = sign(delta_x_turn); 
    elseif abs(wrapToPi(theta0_actual - (-pi/2))) < pi/4, turn_direction_k = -sign(delta_x_turn); 
    else, vec_P0_P3 = P3_target - P0_actual; cross_prod_z_heuristic = dir_P0(1)*vec_P0_P3(2) - dir_P0(2)*vec_P0_P3(1); turn_direction_k = sign(cross_prod_z_heuristic); end
    if turn_direction_k == 0, turn_direction_k = 1; end 
    perp_P0 = [-dir_P0(2), dir_P0(1)]; P1 = P0_actual + d1_long * dir_P0 + (d1_lat * turn_direction_k) * perp_P0;
    dir_P3_entry = [cos(theta3_target_entry), sin(theta3_target_entry)];
    perp_P3 = [-dir_P3_entry(2), dir_P3_entry(1)]; P2 = P3_target - d2_long * dir_P3_entry + (d2_lat * turn_direction_k) * perp_P3;
    control_points_struct = struct('P0', P0_actual, 'P1', P1, 'P2', P2, 'P3', P3_target);
    est_len = norm(P1-P0_actual) + norm(P2-P1) + norm(P3_target-P2); num_bpts = ceil(est_len * points_per_meter); num_bpts = max(num_bpts, 40);
    t_param = linspace(0,1, num_bpts)';
    x_bezier = (1-t_param).^3.*P0_actual(1) + 3.*(1-t_param).^2.*t_param.*P1(1) + 3.*(1-t_param).*t_param.^2.*P2(1) + t_param.^3.*P3_target(1);
    y_bezier = (1-t_param).^3.*P0_actual(2) + 3.*(1-t_param).^2.*t_param.*P1(2) + 3.*(1-t_param).*t_param.^2.*P2(2) + t_param.^3.*P3_target(2);
    x_bezier = x_bezier'; y_bezier = y_bezier';
    if nargin > 7 && isgraphics(ax_to_plot_on) && ~isempty(ax_to_plot_on) && any(ishandle(ax_to_plot_on))
        current_plots_in_ax = findobj(ax_to_plot_on, 'Type', 'Line', '-or', 'Type', 'Text'); % Text also for P0, P1 labels if any
        debug_plot_tags = {'debug_P0','debug_P1','debug_P2','debug_P3','debug_L01','debug_L12','debug_L23'};
        for k_tag = 1:length(debug_plot_tags), delete(findobj(current_plots_in_ax, 'Tag', debug_plot_tags{k_tag})); end
        hold(ax_to_plot_on, 'on');
        h_debug_plots(1)=plot(ax_to_plot_on,P0_actual(1),P0_actual(2),'ko','MarkerSize',6,'MarkerFaceColor','k','DisplayName','P0 (Bz Start)','Tag','debug_P0');
        h_debug_plots(2)=plot(ax_to_plot_on,P1(1),P1(2),'gx','MarkerSize',10,'LineWidth',2,'DisplayName','P1 (Bz Ctrl)','Tag','debug_P1');
        h_debug_plots(3)=plot(ax_to_plot_on,P2(1),P2(2),'bx','MarkerSize',10,'LineWidth',2,'DisplayName','P2 (Bz Ctrl)','Tag','debug_P2');
        h_debug_plots(4)=plot(ax_to_plot_on,P3_target(1),P3_target(2),'ro','MarkerSize',6,'MarkerFaceColor','r','DisplayName','P3 (Bz End)','Tag','debug_P3');
        h_debug_plots(5)=plot(ax_to_plot_on,[P0_actual(1) P1(1)],[P0_actual(2) P1(2)],'g:','HandleVisibility','off','Tag','debug_L01');
        h_debug_plots(6)=plot(ax_to_plot_on,[P1(1) P2(1)],[P1(2) P2(2)],'k:','HandleVisibility','off','Tag','debug_L12');
        h_debug_plots(7)=plot(ax_to_plot_on,[P2(1) P3_target(1)],[P2(2) P3_target(2)],'b:','HandleVisibility','off','Tag','debug_L23');
        fprintf('DEBUG Bezier CPs Plotted: P0(%.1f,%.1f) P1(%.1f,%.1f) P2(%.1f,%.1f) P3(%.1f,%.1f)\n', P0_actual, P1, P2, P3_target);
    end
end

% --- simulate_vehicle_from_pose (Your provided function) ---
function [truck_s, trailer_s, target_xs_out, target_ys_out] = simulate_vehicle_from_pose(...
    xr_s, yr_s, ...                       
    x0_truck_geom, y0_truck_geom, th0_truck_geom_rad, ... 
    x0_trailer_geom, y0_trailer_geom, th0_trailer_geom_rad, ... 
    g_geom, RO_truck_param, RO_trailer_param, segment_type_str)

    n_s_ref = length(xr_s); 
    truck_s.x=[]; truck_s.y=[]; truck_s.theta=[]; trailer_s.x=[]; trailer_s.y=[]; trailer_s.theta=[]; target_xs_out = []; target_ys_out = [];
    if n_s_ref < 2, fprintf('Simulate_vehicle: Ref path too short.\n'); truck_s.x=x0_truck_geom; truck_s.y=y0_truck_geom; truck_s.theta=th0_truck_geom_rad; trailer_s.x=x0_trailer_geom; trailer_s.y=y0_trailer_geom; trailer_s.theta=th0_trailer_geom_rad; if n_s_ref==1,target_xs_out=xr_s(1);target_ys_out=yr_s(1);end; return; end
    req_fields={'WB_truck','look','v','dt','Ltr','Ltl','Lh'}; for f=1:length(req_fields),if ~isfield(g_geom,req_fields{f})||isempty(g_geom.(req_fields{f})),error('g_geom.%s missing.',req_fields{f});end;end; if g_geom.v<=0||g_geom.dt<=0,error('g_geom.v/dt must be positive.');end
    path_dx=diff(xr_s); path_dy=diff(yr_s); est_path_len=sum(sqrt(path_dx.^2+path_dy.^2));
    ideal_steps=ceil(est_path_len/(g_geom.v*g_geom.dt)); max_sim_steps=ideal_steps*2+100; max_sim_steps=max(max_sim_steps,n_s_ref+20); max_sim_steps=min(max_sim_steps,15000);
    truck_s_hist=struct('x',zeros(max_sim_steps,1),'y',zeros(max_sim_steps,1),'theta',zeros(max_sim_steps,1)); trailer_s_hist=struct('x',zeros(max_sim_steps,1),'y',zeros(max_sim_steps,1),'theta',zeros(max_sim_steps,1));
    target_xs_hist=nan(max_sim_steps,1); target_ys_hist=nan(max_sim_steps,1);
    truck_s_hist.x(1)=x0_truck_geom; truck_s_hist.y(1)=y0_truck_geom; truck_s_hist.theta(1)=th0_truck_geom_rad;
    trailer_s_hist.x(1)=x0_trailer_geom; trailer_s_hist.y(1)=y0_trailer_geom; trailer_s_hist.theta(1)=th0_trailer_geom_rad;
    i_sim_step=1; 
    if nargin>11&&strcmp(segment_type_str,'turn_bezier'),dist_thresh_stop=max(g_geom.v*g_geom.dt*2.0,0.25);else,dist_thresh_stop=max(g_geom.v*g_geom.dt*1.5,0.10);end
    min_dist_end_achieved=inf; stall_cnt=0; max_stall_steps=ceil(g_geom.look/(g_geom.v*g_geom.dt))+20;
    dist_geom_center_to_front_bumper = g_geom.Ltr / 2;
    while i_sim_step < max_sim_steps
        curr_truck_x=truck_s_hist.x(i_sim_step); curr_truck_y=truck_s_hist.y(i_sim_step); curr_truck_th=truck_s_hist.theta(i_sim_step);
        curr_front_bumper_x = curr_truck_x + dist_geom_center_to_front_bumper * cos(curr_truck_th);
        curr_front_bumper_y = curr_truck_y + dist_geom_center_to_front_bumper * sin(curr_truck_th);
        dist_geom_to_ref_end=sqrt((xr_s(end)-curr_truck_x)^2+(yr_s(end)-curr_truck_y)^2); if dist_geom_to_ref_end<dist_thresh_stop,break;end
        dist_sq_geom_path=(xr_s-curr_truck_x).^2+(yr_s-curr_truck_y).^2;[~,idx_cl_geom]=min(dist_sq_geom_path);
        if idx_cl_geom>=n_s_ref-max(5,floor(g_geom.look/(est_path_len/n_s_ref+1e-6))),if dist_geom_to_ref_end<min_dist_end_achieved+0.01,stall_cnt=stall_cnt+1;else,min_dist_end_achieved=dist_geom_to_ref_end;stall_cnt=0;end;if stall_cnt>max_stall_steps,break;end;else,min_dist_end_achieved=inf;stall_cnt=0;end
        dist_sq_fb_path=(xr_s-curr_front_bumper_x).^2+(yr_s-curr_front_bumper_y).^2;[~,idx_cl_fb]=min(dist_sq_fb_path);
        cur_srch=idx_cl_fb;tgt_idx=idx_cl_fb;
        while cur_srch<=n_s_ref,dist_fb_to_path_pt=sqrt((xr_s(cur_srch)-curr_front_bumper_x)^2+(yr_s(cur_srch)-curr_front_bumper_y)^2);if dist_fb_to_path_pt>=g_geom.look,tgt_idx=cur_srch;break;end;if cur_srch==n_s_ref,tgt_idx=n_s_ref;break;end;cur_srch=cur_srch+1;end
        xt=xr_s(tgt_idx);yt=yr_s(tgt_idx);target_xs_hist(i_sim_step)=xt;target_ys_hist(i_sim_step)=yt;
        alpha=wrapToPi(atan2(yt-curr_front_bumper_y,xt-curr_front_bumper_x)-curr_truck_th);
        Le=sqrt((xt-curr_front_bumper_x)^2+(yt-curr_front_bumper_y)^2);Le=max(Le,0.1);
        del=atan2(2*g_geom.WB_truck*sin(alpha),Le);max_steer=deg2rad(40);del=max(-max_steer,min(max_steer,del));
        truck_s_hist.x(i_sim_step+1)=curr_truck_x+g_geom.v*cos(curr_truck_th)*g_geom.dt;truck_s_hist.y(i_sim_step+1)=curr_truck_y+g_geom.v*sin(curr_truck_th)*g_geom.dt;truck_s_hist.theta(i_sim_step+1)=wrapToPi(curr_truck_th+(g_geom.v/g_geom.WB_truck)*tan(del)*g_geom.dt);
        h_x=truck_s_hist.x(i_sim_step+1)-(g_geom.Ltr/2)*cos(truck_s_hist.theta(i_sim_step+1));h_y=truck_s_hist.y(i_sim_step+1)-(g_geom.Ltr/2)*sin(truck_s_hist.theta(i_sim_step+1));
        curr_tr_x=trailer_s_hist.x(i_sim_step);curr_tr_y=trailer_s_hist.y(i_sim_step);
        trailer_s_hist.theta(i_sim_step+1)=wrapToPi(atan2(h_y-curr_tr_y,h_x-curr_tr_x));
        dist_h_to_tr_g=g_geom.Lh+(g_geom.Ltl/2);
        trailer_s_hist.x(i_sim_step+1)=h_x-dist_h_to_tr_g*cos(trailer_s_hist.theta(i_sim_step+1));trailer_s_hist.y(i_sim_step+1)=h_y-dist_h_to_tr_g*sin(trailer_s_hist.theta(i_sim_step+1));
        i_sim_step=i_sim_step+1;
    end
    if i_sim_step==max_sim_steps,final_dist=sqrt((xr_s(end)-truck_s_hist.x(i_sim_step))^2+(yr_s(end)-truck_s_hist.y(i_sim_step))^2);if final_dist>=dist_thresh_stop,warning('Simulate_vehicle: Max sim steps (%d) hit. Dist to ref end: %.2fm.',max_sim_steps,final_dist);end;end
    num_actual_steps=i_sim_step;truck_s.x=truck_s_hist.x(1:num_actual_steps);truck_s.y=truck_s_hist.y(1:num_actual_steps);truck_s.theta=truck_s_hist.theta(1:num_actual_steps);
    trailer_s.x=trailer_s_hist.x(1:num_actual_steps);trailer_s.y=trailer_s_hist.y(1:num_actual_steps);trailer_s.theta=trailer_s_hist.theta(1:num_actual_steps);
    if num_actual_steps>0&&isnan(target_xs_hist(num_actual_steps))&&num_actual_steps>1,target_xs_hist(num_actual_steps)=target_xs_hist(num_actual_steps-1);target_ys_hist(num_actual_steps)=target_ys_hist(num_actual_steps-1);
    elseif num_actual_steps==1,target_xs_hist(1)=xr_s(1);target_ys_hist(1)=yr_s(1);
    elseif num_actual_steps==0,target_xs_out=[];target_ys_out=[];return;end
    target_xs_out=target_xs_hist(1:num_actual_steps);target_ys_out=target_ys_hist(1:num_actual_steps);
end

% --- animate_vehicles_with_map (Your provided function) ---
function animate_vehicles_with_map(map, x_ref, y_ref, truck, trailer, target_xs,target_ys)
    if isempty(truck) || ~isfield(truck,'x') || isempty(truck.x), fprintf('Animation skipped: Truck sim data empty.\n'); return; end
    if ~isempty(target_xs) && isrow(target_xs), target_xs = target_xs'; end; if ~isempty(target_ys) && isrow(target_ys), target_ys = target_ys'; end
    existing_fig = findall(0,'Type','figure','-regexp','Name','Truck-Trailer on Open Field');
    if ~isempty(existing_fig), fig_anim = existing_fig(1); figure(fig_anim); ax_anim = fig_anim.CurrentAxes; cla(ax_anim);
    else, fig_anim = figure('Name','Truck-Trailer on Open Field', 'Color','white', 'Position', [1000 50 800 700]); ax_anim = axes(fig_anim); end
    show(map, 'Parent', ax_anim); hold(ax_anim, 'on'); axis(ax_anim, 'equal'); grid(ax_anim, 'on');
    plot(ax_anim, x_ref, y_ref, 'k:', 'LineWidth', 1, 'DisplayName', 'Reference Path');
    truck_length_vis = 2.2; truck_width_vis = 1.0; trailer_length_vis = 1.0; trailer_width_vis = 1.0;
    truck_patch = patch(ax_anim,'Vertices',zeros(4,2),'Faces',1:4,'FaceColor','blue','EdgeColor','k','FaceAlpha',0.7,'LineWidth',1);
    trailer_patch = patch(ax_anim,'Vertices',zeros(4,2),'Faces',1:4,'FaceColor','red','EdgeColor','k','FaceAlpha',0.7,'LineWidth',1);
    hitch_line = plot(ax_anim, [0 0], [0 0], 'g-', 'LineWidth', 2, 'DisplayName', 'Hitch');
    target_marker_plot = plot(ax_anim, NaN, NaN, 'go', 'MarkerSize', 8, 'MarkerFaceColor', [0.2 0.8 0.2],'LineWidth',1.5, 'DisplayName', 'PP Target');
    legend(ax_anim, 'Location', 'eastoutside', 'AutoUpdate', 'off');
    num_sim_steps = length(truck.x); if num_sim_steps == 0, return; end
    target_frames_for_anim = 150; anim_step = max(1, floor(num_sim_steps / target_frames_for_anim));
    for i = 1:anim_step:num_sim_steps
        if ~isempty(target_xs)&&i<=length(target_xs)&&~isnan(target_xs(i)),set(target_marker_plot,'XData',target_xs(i),'YData',target_ys(i));else,set(target_marker_plot,'XData',NaN,'YData',NaN);end
        set(truck_patch, 'Vertices', get_rectangle_for_anim(truck.x(i),truck.y(i),truck.theta(i),truck_length_vis,truck_width_vis));
        if isfield(trailer,'x')&&~isempty(trailer.x)&&i<=length(trailer.x)&&~isempty(trailer.x(i))&&~isnan(trailer.x(i))
            set(trailer_patch,'Vertices',get_rectangle_for_anim(trailer.x(i),trailer.y(i),trailer.theta(i),trailer_length_vis,trailer_width_vis));
            h_trk_r_x=truck.x(i)-(truck_length_vis/2)*cos(truck.theta(i));h_trk_r_y=truck.y(i)-(truck_length_vis/2)*sin(truck.theta(i));
            tr_frnt_x=trailer.x(i)+(trailer_length_vis/2)*cos(trailer.theta(i));tr_frnt_y=trailer.y(i)+(trailer_length_vis/2)*sin(trailer.theta(i));
            set(hitch_line,'XData',[h_trk_r_x tr_frnt_x],'YData',[h_trk_r_y tr_frnt_y]);
        else,set(trailer_patch,'Vertices',nan(4,2));set(hitch_line,'XData',[NaN NaN],'YData',[NaN NaN]);end
        drawnow limitrate; pause(0.005); 
    end
    if ~isempty(truck.x)&&~isempty(truck.x(end))&&~isnan(truck.x(end))
        final_idx=length(truck.x);
        if ~isempty(target_xs)&&final_idx<=length(target_xs)&&~isnan(target_xs(final_idx)),set(target_marker_plot,'XData',target_xs(final_idx),'YData',target_ys(final_idx));else,set(target_marker_plot,'XData',NaN,'YData',NaN);end
        set(truck_patch,'Vertices',get_rectangle_for_anim(truck.x(final_idx),truck.y(final_idx),truck.theta(final_idx),truck_length_vis,truck_width_vis));
        if isfield(trailer,'x')&&~isempty(trailer.x)&&final_idx<=length(trailer.x)&&~isempty(trailer.x(final_idx))&&~isnan(trailer.x(final_idx))
            set(trailer_patch,'Vertices',get_rectangle_for_anim(trailer.x(final_idx),trailer.y(final_idx),trailer.theta(final_idx),trailer_length_vis,trailer_width_vis));
            h_trk_r_x=truck.x(final_idx)-(truck_length_vis/2)*cos(truck.theta(final_idx));h_trk_r_y=truck.y(final_idx)-(truck_length_vis/2)*sin(truck.theta(final_idx));
            tr_frnt_x=trailer.x(final_idx)+(trailer_length_vis/2)*cos(trailer.theta(final_idx));tr_frnt_y=trailer.y(final_idx)+(trailer_length_vis/2)*sin(trailer.theta(final_idx));
            set(hitch_line,'XData',[h_trk_r_x tr_frnt_x],'YData',[h_trk_r_y tr_frnt_y]);
        else,set(trailer_patch,'Vertices',nan(4,2));set(hitch_line,'XData',[NaN NaN],'YData',[NaN NaN]);end
        drawnow;
    end
end

% --- get_rectangle_for_anim ---
function rect = get_rectangle_for_anim(x_center, y_center, theta_rad, L, W)
    R_matrix = [cos(theta_rad) -sin(theta_rad); sin(theta_rad)  cos(theta_rad)];
    corners_local = [-L/2 -W/2; L/2 -W/2; L/2 W/2; -L/2 W/2]; 
    rect = (R_matrix * corners_local')' + repmat([x_center y_center],4,1);
end