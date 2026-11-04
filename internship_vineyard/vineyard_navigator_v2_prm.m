% vineyardNavigator_SegmentBySegment.m
% Main script for step-by-step path planning and execution in a vineyard,
% integrating PRM for custom point navigation and specific logic for row/headland paths.

clear; clc;
% close all; 

fprintf('--- Vineyard Navigation System (Segment-by-Segment with PRM) ---\n');

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

% --- 1.5 Initialize PRM (Done ONCE) ---
fprintf('Initializing PRM...\n');
planningMap = copy(environmentMap); 
prmInflateRadius = 0.40; 
inflate(planningMap, prmInflateRadius);
[prm_nodes_initial, prm_adjacency_initial] = buildPRMRoadmap(planningMap, environmentMap, 500, 20); 
if isempty(prm_nodes_initial)
    error('PRM Initialization Failed: No nodes sampled. Adjust parameters or map.');
end
fprintf('PRM: %d nodes and %d edges built.\n', size(prm_nodes_initial,1), nnz(prm_adjacency_initial)/2);
plot(ax_main, prm_nodes_initial(:,1), prm_nodes_initial(:,2), 'c.', 'MarkerSize', 6, 'DisplayName', 'PRM Nodes'); 


% --- 2. Define Snap Points, Custom Points & Task Sequence ---
all_snap_points = collectAllSnapPoints(UPDATED_VINEYARD_PARAMS);
custom_points_list = { ...
    {'name','custom_point_A', 'pose', [43, 25, deg2rad(90)]}, ... 
    {'name','custom_point_B', 'pose', [21, 28, deg2rad(-90)]}  
};
all_custom_points_struct = struct('name',{},'xy',{},'theta_rad',{});
for k=1:length(custom_points_list)
    all_custom_points_struct(k).name = custom_points_list{k}{2};
    all_custom_points_struct(k).xy = custom_points_list{k}{4}(1:2);
    all_custom_points_struct(k).theta_rad = custom_points_list{k}{4}(3);
end

task_sequence_with_custom = { ... 
    {'type', 'snap', 'name', 'Row_1_BottomEntry'}, ...
    {'type', 'snap', 'name', 'Row_1_TopExit'}, ...
    {'type', 'snap', 'name', 'Row_2_TopEntry'}, ...
    {'type', 'snap', 'name', 'Row_2_BottomExit'}, ...
    {'type', 'snap', 'name', 'Row_3_BottomEntry'}, ...
    {'type', 'snap', 'name', 'Row_3_TopExit'}, ...
    {'type', 'snap', 'name', 'Row_1_TopEntry'}, ...   
    {'type', 'snap', 'name', 'Row_1_BottomExit'}, ...
    {'type', 'custom', 'name', 'custom_point_A'}, ... 
    {'type', 'custom', 'name', 'custom_point_B'}, ... 
};
task_waypoints_full_sequence = getWaypointsFromMixedTask(task_sequence_with_custom, all_snap_points, all_custom_points_struct);

for i=1:length(task_waypoints_full_sequence) 
    plot_color = 'yd'; marker_face_color = 'y';
    if strcmp(task_waypoints_full_sequence(i).waypoint_type, 'custom'), plot_color = 'cd'; marker_face_color = 'c'; end
    plot(ax_main, task_waypoints_full_sequence(i).xy(1), task_waypoints_full_sequence(i).xy(2), plot_color, 'MarkerFaceColor', marker_face_color, 'MarkerSize', 7, 'DisplayName', 'Task Waypoint');
    text(ax_main, task_waypoints_full_sequence(i).xy(1)+0.3, task_waypoints_full_sequence(i).xy(2), strrep(task_waypoints_full_sequence(i).name, '_', ' '), 'Color', 'blue', 'FontSize', 8);
end
legend(ax_main, 'off'); legend(ax_main, 'show', 'Location', 'southoutside');

% --- 3. Initialize Vehicle State & Full Path Accumulation Variables ---
current_vehicle_pose_world = [task_waypoints_full_sequence(1).xy, task_waypoints_full_sequence(1).theta_rad]; 
prev_trailer_sim_end_theta = current_vehicle_pose_world(3); 
x_path_full_executed_ref = []; y_path_full_executed_ref = [];

% --- 4. Centralized Vehicle Definitions & Costmaps ---
current_geom = geom(); 
current_geom.dt = 0.05; current_geom.v = 0.5; current_geom.look = 0.7; 
current_geom.WB_truck = current_geom.Ltr * 0.70; current_geom.WB_trailer = current_geom.Ltl * 0.70; 
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
    fprintf('\n--- Segment %d: From %s (%s) to %s (%s) ---\n', seg_idx, wp_start_details.name, wp_start_details.waypoint_type, wp_end_details.name, wp_end_details.waypoint_type);
    fprintf('Vehicle current start: X=%.2f, Y=%.2f, Th=%.2f rad. Prev Trailer Th=%.2f\n', current_vehicle_pose_world(1), current_vehicle_pose_world(2), current_vehicle_pose_world(3), prev_trailer_sim_end_theta);

    if ~isempty(h_plots_on_main_ax)
        valid_h = isgraphics(h_plots_on_main_ax); 
        if any(valid_h), delete(h_plots_on_main_ax(valid_h)); end
    end
    h_plots_on_main_ax = gobjects(0); 

    actual_vehicle_start_pose_for_segment_xy_th = current_vehicle_pose_world; 
    [x_seg_ref, y_seg_ref, theta_seg_ref, segment_type, h_bezier_debug_plots_on_main] = ...
        generateSingleSegmentPath_V2(wp_start_details, actual_vehicle_start_pose_for_segment_xy_th, wp_end_details, ...
                                   UPDATED_VINEYARD_PARAMS, ax_main, planningMap, prm_nodes_initial, prm_adjacency_initial); 
    h_plots_on_main_ax = [h_plots_on_main_ax, h_bezier_debug_plots_on_main(isgraphics(h_bezier_debug_plots_on_main))]; 

    if isempty(x_seg_ref) || length(x_seg_ref) < 2, fprintf('Seg %d path gen failed/short. Skipping.\n', seg_idx); input('Error. Enter...'); continue; end
    
    fprintf('INFO: For Segment %d (%s):\n', seg_idx, segment_type);
    fprintf('  Actual vehicle start: X=%.2f, Y=%.2f, Theta=%.2f rad\n', actual_vehicle_start_pose_for_segment_xy_th);
    fprintf('  Generated Ref Path (x_seg_ref) has %d points.\n', length(x_seg_ref));
    if ~isempty(x_seg_ref), fprintf('  Ref Path Start:X=%.2f,Y=%.2f,Th=%.2f | End:X=%.2f,Y=%.2f,Th=%.2f\n',x_seg_ref(1),y_seg_ref(1),theta_seg_ref(1),x_seg_ref(end),y_seg_ref(end),theta_seg_ref(end)); end
    fprintf('  Target Waypoint: %s at X=%.2f,Y=%.2f,Th=%.2f\n', wp_end_details.name, wp_end_details.xy(1),wp_end_details.xy(2),wp_end_details.theta_rad);

    h_current_segment_plot = plot(ax_main, x_seg_ref, y_seg_ref, 'c--', 'LineWidth', 1.5, 'DisplayName', 'Current Segment (Initial Ref)');
    h_plots_on_main_ax(end+1) = h_current_segment_plot; 
    legend(ax_main, 'off'); legend(ax_main, 'show', 'Location', 'southoutside'); 
    input(sprintf('Segment %d ref path generated. Press Enter to simulate...', seg_idx), 's');
    
    x0_t=actual_vehicle_start_pose_for_segment_xy_th(1); y0_t=actual_vehicle_start_pose_for_segment_xy_th(2); th0_t_rad=actual_vehicle_start_pose_for_segment_xy_th(3);
    th0_tr_rad = prev_trailer_sim_end_theta;
    hitch_x_curr = x0_t-(current_geom.Ltr/2)*cos(th0_t_rad); hitch_y_curr = y0_t-(current_geom.Ltr/2)*sin(th0_t_rad);
    dist_h_to_tr_geom = current_geom.Lh+(current_geom.Ltl/2);
    x0_tr = hitch_x_curr-dist_h_to_tr_geom*cos(th0_tr_rad); y0_tr = hitch_y_curr-dist_h_to_tr_geom*sin(th0_tr_rad);
    [truck_sim_seg_orig, trailer_sim_seg_orig, target_xs_orig_seg, target_ys_orig_seg] = simulate_vehicle_from_pose(x_seg_ref,y_seg_ref,x0_t,y0_t,th0_t_rad,x0_tr,y0_tr,th0_tr_rad,current_geom,RO_truck_param,RO_trailer_param, segment_type);
    
    if ~isempty(truck_sim_seg_orig.x), animate_vehicles_with_map(environmentMap,x_seg_ref,y_seg_ref,truck_sim_seg_orig,trailer_sim_seg_orig,target_xs_orig_seg,target_ys_orig_seg);
        anim_fig_h = findall(0,'Type','figure','-regexp','Name','Truck-Trailer on Open Field'); if ~isempty(anim_fig_h),title(anim_fig_h(1).CurrentAxes,sprintf('Anim: Seg %d (Initial): %s to %s',seg_idx,strrep(wp_start_details.name,'_',' '),strrep(wp_end_details.name,'_',' ')));end
    else, fprintf('Skipping anim: initial sim seg %d empty.\n', seg_idx); end
    input(sprintf('Seg %d initial animation complete. Press Enter...', seg_idx), 's');

    x_seg_mod=x_seg_ref; y_seg_mod=y_seg_ref; theta_seg_mod=theta_seg_ref; 
    truck_sim_seg_final=truck_sim_seg_orig; trailer_sim_seg_final=trailer_sim_seg_orig;
    target_xs_final_seg = target_xs_orig_seg; target_ys_final_seg = target_ys_orig_seg;

    RUN_MODIFY_CURVES = false; 
    if (strcmp(segment_type, 'turn_bezier') || strcmp(segment_type, 'prm_custom') || strcmp(segment_type, 'prm_default')) ...
            && ~isempty(truck_sim_seg_orig.x) && RUN_MODIFY_CURVES
        fprintf('Seg %d type %s. Applying Recker-Bezier...\n', seg_idx, segment_type);
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
            x0_t_mod=actual_vehicle_start_pose_for_segment_xy_th(1);y0_t_mod=actual_vehicle_start_pose_for_segment_xy_th(2);th0_t_rad_mod=actual_vehicle_start_pose_for_segment_xy_th(3);
            th0_tr_rad_mod = prev_trailer_sim_end_theta;
            hitch_x_mod = x0_t_mod-(current_geom.Ltr/2)*cos(th0_t_rad_mod); hitch_y_mod = y0_t_mod-(current_geom.Ltr/2)*sin(th0_t_rad_mod);
            dist_h_to_tr_geom_mod = current_geom.Lh+(current_geom.Ltl/2);
            x0_tr_mod = hitch_x_mod-dist_h_to_tr_geom_mod*cos(th0_tr_rad_mod); y0_tr_mod = hitch_y_mod-dist_h_to_tr_geom_mod*sin(th0_tr_rad_mod);
            [truck_sim_seg_final, trailer_sim_seg_final, target_xs_final_seg, target_ys_final_seg] = simulate_vehicle_from_pose(x_seg_mod,y_seg_mod,x0_t_mod,y0_t_mod,th0_t_rad_mod,x0_tr_mod,y0_tr_mod,th0_tr_rad_mod,current_geom,RO_truck_param,RO_trailer_param, segment_type); 
        else
            fprintf('!!! Seg %d Recker-Bezier mod FAILED. Using initial. !!!\n', seg_idx);
        end
        if ~isempty(truck_sim_seg_final.x)
            animate_vehicles_with_map(environmentMap, x_seg_mod, y_seg_mod, truck_sim_seg_final, trailer_sim_seg_final, target_xs_final_seg, target_ys_final_seg);
            anim_fig_h = findall(0,'Type','figure','-regexp','Name','Truck-Trailer on Open Field');
            if ~isempty(anim_fig_h), title(anim_fig_h(1).CurrentAxes, sprintf('Anim: Seg %d (Final): %s to %s', seg_idx, strrep(wp_start_details.name,'_',' '), strrep(wp_end_details.name,'_',' '))); end
        else, fprintf('Skipping anim: final sim seg %d empty.\n', seg_idx); end
        input(sprintf('Seg %d final anim complete. Press Enter...', seg_idx), 's');
    elseif (strcmp(segment_type, 'turn_bezier') || strcmp(segment_type, 'prm_custom') || strcmp(segment_type, 'prm_default')) && ~RUN_MODIFY_CURVES
         fprintf('Skipped Recker-Bezier for segment %d (type: %s).\n', seg_idx, segment_type);
    end

    final_executed_ref_x_seg = x_seg_mod; final_executed_ref_y_seg = y_seg_mod;
    if isempty(x_path_full_executed_ref),x_path_full_executed_ref=final_executed_ref_x_seg;y_path_full_executed_ref=final_executed_ref_y_seg;
    else,x_path_full_executed_ref=[x_path_full_executed_ref,final_executed_ref_x_seg(2:end)];y_path_full_executed_ref=[y_path_full_executed_ref,final_executed_ref_y_seg(2:end)];end
    
    last_valid_truck_sim=[]; last_valid_trailer_sim=[];
    if ~isempty(truck_sim_seg_final)&&~isempty(truck_sim_seg_final.x),last_valid_truck_sim=truck_sim_seg_final;last_valid_trailer_sim=trailer_sim_seg_final;
    elseif ~isempty(truck_sim_seg_orig)&&~isempty(truck_sim_seg_orig.x),last_valid_truck_sim=truck_sim_seg_orig;last_valid_trailer_sim=trailer_sim_seg_orig;end

    if ~isempty(last_valid_truck_sim)&&isfield(last_valid_truck_sim,'x')&&~isempty(last_valid_truck_sim.x)
        current_vehicle_pose_world=[last_valid_truck_sim.x(end),last_valid_truck_sim.y(end),last_valid_truck_sim.theta(end)];
        if ~isempty(last_valid_trailer_sim)&&isfield(last_valid_trailer_sim,'theta')&&~isempty(last_valid_trailer_sim.theta),prev_trailer_sim_end_theta=last_valid_trailer_sim.theta(end);
        else,prev_trailer_sim_end_theta=current_vehicle_pose_world(3);fprintf('Warn: Trailer sim theta empty seg %d end.\n',seg_idx);end
    else
        fprintf('CRITICAL WARN: No valid sim data for seg %d. Using geometric end.\n',seg_idx);
        if ~isempty(theta_seg_mod),current_vehicle_pose_world=[final_executed_ref_x_seg(end),final_executed_ref_y_seg(end),theta_seg_mod(end)];prev_trailer_sim_end_theta=theta_seg_mod(end);
        elseif ~isempty(theta_seg_ref),current_vehicle_pose_world=[final_executed_ref_x_seg(end),final_executed_ref_y_seg(end),theta_seg_ref(end)];prev_trailer_sim_end_theta=theta_seg_ref(end);
        else,current_vehicle_pose_world=[final_executed_ref_x_seg(end),final_executed_ref_y_seg(end),current_vehicle_pose_world(3)];prev_trailer_sim_end_theta=current_vehicle_pose_world(3);end
    end
    fprintf('Vehicle pose after seg %d: X=%.2f,Y=%.2f,Th=%.2f rad. PrevTrailerTh=%.2f\n',seg_idx,current_vehicle_pose_world(1),current_vehicle_pose_world(2),current_vehicle_pose_world(3),prev_trailer_sim_end_theta);
end
fprintf('\n--- All Segments Processed ---\n');
figure(fig_main_handle);
plot(ax_main,x_path_full_executed_ref,y_path_full_executed_ref,'m.-','LineWidth',1.2,'DisplayName','Full Planned Path (Concatenated Ref Segments)');
legend(ax_main,'off');legend(ax_main,'show','Location','southoutside'); title(ax_main,'Full Path Execution Plan');

% ====== HELPER FUNCTIONS ======
% (All other helper functions like defineVineyardParameters, collectAllSnapPoints, 
%  getWaypointsFromMixedTask, generateInitialHeadlandBezier_V2, geom, 
%  simulate_vehicle_from_pose, animate_vehicles_with_map, get_rectangle_for_anim
%  MUST BE DEFINED HERE OR ACCESSIBLE ON THE MATLAB PATH)
%  The PRM functions buildPRMRoadmap and findPathPRM are included below.

function VINEYARD_PARAMS = defineVineyardParameters()
    VINEYARD_PARAMS.field_x_limits = [1, 49]; 
    VINEYARD_PARAMS.bottom_headland_y_limits = [2.3, 6]; 
    VINEYARD_PARAMS.top_headland_y_limits = [44.5, 48];    
    VINEYARD_PARAMS.row_y_start = VINEYARD_PARAMS.bottom_headland_y_limits(2)+0.5; 
    VINEYARD_PARAMS.row_y_end = VINEYARD_PARAMS.top_headland_y_limits(1)-0.5;     
    VINEYARD_PARAMS.row_y_bottom_snap = VINEYARD_PARAMS.bottom_headland_y_limits(2) + 0.5; 
    VINEYARD_PARAMS.row_y_top_snap    = VINEYARD_PARAMS.top_headland_y_limits(1) - 0.5;
    row_centers_x = [3, 7, 10.75]; VINEYARD_PARAMS.rows = []; % Adjusted Row 3 center slightly for filari.png
    for i = 1:length(row_centers_x)
        VINEYARD_PARAMS.rows(i).name = sprintf('Row_%d', i); VINEYARD_PARAMS.rows(i).center_x = row_centers_x(i);
        VINEYARD_PARAMS.rows(i).bottom_entry_xy = [row_centers_x(i), VINEYARD_PARAMS.row_y_bottom_snap]; VINEYARD_PARAMS.rows(i).bottom_entry_theta_rad = pi/2; 
        VINEYARD_PARAMS.rows(i).top_exit_xy = [row_centers_x(i), VINEYARD_PARAMS.row_y_top_snap]; VINEYARD_PARAMS.rows(i).top_exit_theta_rad = pi/2;  
        VINEYARD_PARAMS.rows(i).top_entry_xy = [row_centers_x(i), VINEYARD_PARAMS.row_y_top_snap]; VINEYARD_PARAMS.rows(i).top_entry_theta_rad = -pi/2; 
        VINEYARD_PARAMS.rows(i).bottom_exit_xy = [row_centers_x(i), VINEYARD_PARAMS.row_y_bottom_snap]; VINEYARD_PARAMS.rows(i).bottom_exit_theta_rad = -pi/2;
    end
    if length(VINEYARD_PARAMS.rows) >= 2, VINEYARD_PARAMS.row_width_center_to_center = VINEYARD_PARAMS.rows(2).center_x - VINEYARD_PARAMS.rows(1).center_x; else, VINEYARD_PARAMS.row_width_center_to_center = 4; end
    VINEYARD_PARAMS.parking_spots = []; 
end

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

function waypoints = getWaypointsFromMixedTask(task_sequence_cell_array, all_snap_points, all_custom_points)
    waypoints = struct('name', {}, 'xy', {}, 'theta_rad', {}, 'waypoint_type', {}, 'snap_type', {}, 'theta_rad_implicit_exit', {});
    if isempty(task_sequence_cell_array), warning('Task sequence is empty.'); return; end
    for i = 1:length(task_sequence_cell_array)
        task_item = task_sequence_cell_array{i}; item_type = task_item{2}; item_name = task_item{4};
        waypoints(i).waypoint_type = item_type; waypoints(i).name = item_name;
        if strcmp(item_type, 'snap'), idx = find(strcmp({all_snap_points.name}, item_name));
            if isempty(idx), error('Snap point name "%s" not found.', item_name); end
            waypoints(i).xy = all_snap_points(idx).xy; waypoints(i).theta_rad = all_snap_points(idx).theta_rad;
            if contains(item_name,'Parking'),waypoints(i).snap_type='parking';elseif contains(item_name,'Entry'),waypoints(i).snap_type='row_entry';elseif contains(item_name,'Exit'),waypoints(i).snap_type='row_exit';else,waypoints(i).snap_type='unknown_snap';end
        elseif strcmp(item_type, 'custom'), idx = find(strcmp({all_custom_points.name}, item_name));
            if isempty(idx), error('Custom point name "%s" not found.', item_name); end
            waypoints(i).xy = all_custom_points(idx).xy; waypoints(i).theta_rad = all_custom_points(idx).theta_rad; waypoints(i).snap_type = 'custom'; 
        else, error('Unknown waypoint type: %s', item_type); end
    end
    for i=1:length(waypoints)-1,p_start=waypoints(i).xy; p_end=waypoints(i+1).xy; waypoints(i).theta_rad_implicit_exit=atan2(p_end(2)-p_start(2),p_end(1)-p_start(1)); end
    if ~isempty(waypoints),waypoints(end).theta_rad_implicit_exit=waypoints(end).theta_rad;end
end

function [x_seg, y_seg, theta_seg, segment_type_str, h_bezier_debug_plots] = generateSingleSegmentPath_V2(...
    wp_start_details, actual_vehicle_start_pose_xy_theta, wp_end_details, ...
    VINEYARD_PARAMS, ax_for_debug_plot, prm_planning_map, prm_nodes, prm_adjacency)
    h_bezier_debug_plots = gobjects(0); 
    P0_actual = actual_vehicle_start_pose_xy_theta(1:2); Theta0_actual = actual_vehicle_start_pose_xy_theta(3); 
    P3_target_xy = wp_end_details.xy; Theta3_target_orientation = wp_end_details.theta_rad; 
    segment_type_str = 'prm_default'; start_is_snap = strcmp(wp_start_details.waypoint_type, 'snap'); end_is_snap = strcmp(wp_end_details.waypoint_type, 'snap');
    if start_is_snap && end_is_snap
        if (strcmp(wp_start_details.snap_type, 'row_entry') && strcmp(wp_end_details.snap_type, 'row_exit'))
            start_row_name_parts = strsplit(wp_start_details.name, '_'); end_row_name_parts = strsplit(wp_end_details.name, '_');
            if strcmp(start_row_name_parts{1},end_row_name_parts{1})&&strcmp(start_row_name_parts{2},end_row_name_parts{2}), segment_type_str = 'in_row'; end
        elseif (strcmp(wp_start_details.snap_type, 'row_exit') && strcmp(wp_end_details.snap_type, 'row_entry'))
            start_row_name_parts = strsplit(wp_start_details.name, '_'); end_row_name_parts = strsplit(wp_end_details.name, '_');
            if ~strcmp(start_row_name_parts{2}, end_row_name_parts{2}), segment_type_str = 'turn_bezier'; end
        end
    end
    if strcmp(wp_start_details.waypoint_type, 'custom') || strcmp(wp_end_details.waypoint_type, 'custom'), segment_type_str = 'prm_custom'; end
    x_seg = []; y_seg = []; points_per_meter_straight = 20; points_per_meter_turn = 20; points_per_meter_prm = 10;
    switch segment_type_str
        case 'in_row', dist_seg = norm(P3_target_xy - P0_actual); num_pts = ceil(dist_seg*points_per_meter_straight); num_pts=max(2,num_pts); x_seg=linspace(P0_actual(1),P3_target_xy(1),num_pts); y_seg=linspace(P0_actual(2),P3_target_xy(2),num_pts);
        case 'turn_bezier'
            if P0_actual(2)>(VINEYARD_PARAMS.bottom_headland_y_limits(2)+VINEYARD_PARAMS.top_headland_y_limits(1))/2,head_lims=VINEYARD_PARAMS.top_headland_y_limits;else,head_lims=VINEYARD_PARAMS.bottom_headland_y_limits;end; head_depth=abs(head_lims(2)-head_lims(1));
            [x_seg,y_seg,~,h_bezier_debug_plots] = generateInitialHeadlandBezier_V2(P0_actual,Theta0_actual,P3_target_xy,Theta3_target_orientation,VINEYARD_PARAMS,head_depth,points_per_meter_turn,ax_for_debug_plot);
        case {'prm_default', 'prm_custom'}
            fprintf('  Using PRM for segment: %s to %s\n', wp_start_details.name, wp_end_details.name);
            [path_nodes_prm, success] = findPathPRM(P0_actual, P3_target_xy, prm_nodes, prm_adjacency, prm_planning_map);
            if success && ~isempty(path_nodes_prm)
                if size(path_nodes_prm,1)==1, x_seg=path_nodes_prm(1,1); y_seg=path_nodes_prm(1,2);
                else, path_dist=[0;cumsum(sqrt(sum(diff(path_nodes_prm,1,1).^2,2)))];num_interp=ceil(path_dist(end)*points_per_meter_prm);num_interp=max(2,num_interp);interp_dist=linspace(0,path_dist(end),num_interp);x_seg=interp1(path_dist,path_nodes_prm(:,1),interp_dist,'pchip');y_seg=interp1(path_dist,path_nodes_prm(:,2),interp_dist,'pchip');end
            else, fprintf('  PRM failed. Fallback to straight line.\n'); segment_type_str='straight_fallback_prm'; dist_seg=norm(P3_target_xy-P0_actual);num_pts=ceil(dist_seg*points_per_meter_straight);num_pts=max(2,num_pts);x_seg=linspace(P0_actual(1),P3_target_xy(1),num_pts);y_seg=linspace(P0_actual(2),P3_target_xy(2),num_pts);end
        otherwise, fprintf('  Unknown type. Fallback to straight line.\n'); segment_type_str='straight_fallback_unknown'; dist_seg=norm(P3_target_xy-P0_actual);num_pts=ceil(dist_seg*points_per_meter_straight);num_pts=max(2,num_pts);x_seg=linspace(P0_actual(1),P3_target_xy(1),num_pts);y_seg=linspace(P0_actual(2),P3_target_xy(2),num_pts);
    end
    if ~isempty(x_seg)&&length(x_seg)>1,dx=diff(x_seg);dy=diff(y_seg);th_calc=atan2(dy,dx);theta_seg=[th_calc(:)',th_calc(end)];theta_seg=unwrap(theta_seg);
    elseif ~isempty(x_seg)&&length(x_seg)==1,theta_seg=Theta0_actual;else,theta_seg=[];x_seg=[];y_seg=[];fprintf('Warn: Empty path seg: %s to %s (Type: %s).\n',wp_start_details.name,wp_end_details.name,segment_type_str);end
end

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
    if abs(wrapToPi(theta0_actual - pi/2)) < pi/4, turn_dir_k = sign(delta_x_turn); 
    elseif abs(wrapToPi(theta0_actual - (-pi/2))) < pi/4, turn_dir_k = -sign(delta_x_turn); 
    else, vec_P0_P3=P3_target-P0_actual; cz=dir_P0(1)*vec_P0_P3(2)-dir_P0(2)*vec_P0_P3(1); turn_dir_k=sign(cz); end 
    if turn_dir_k == 0, turn_dir_k = 1; end 
    perp_P0 = [-dir_P0(2)*turn_dir_k, dir_P0(1)*turn_dir_k]; P1 = P0_actual + d1_long * dir_P0 + d1_lat * perp_P0; 
    dir_P3_entry = [cos(theta3_target_entry), sin(theta3_target_entry)];
    perp_P3 = [-dir_P3_entry(2)*turn_dir_k, dir_P3_entry(1)*turn_dir_k]; P2 = P3_target - d2_long * dir_P3_entry + d2_lat * perp_P3;
    control_points_struct = struct('P0', P0_actual, 'P1', P1, 'P2', P2, 'P3', P3_target);
    est_len = norm(P1-P0_actual) + norm(P2-P1) + norm(P3_target-P2); num_bpts = ceil(est_len * points_per_meter); num_bpts = max(num_bpts, 40);
    t_param = linspace(0,1, num_bpts)';
    x_bezier = (1-t_param).^3.*P0_actual(1) + 3.*(1-t_param).^2.*t_param.*P1(1) + 3.*(1-t_param).*t_param.^2.*P2(1) + t_param.^3.*P3_target(1);
    y_bezier = (1-t_param).^3.*P0_actual(2) + 3.*(1-t_param).^2.*t_param.*P1(2) + 3.*(1-t_param).*t_param.^2.*P2(2) + t_param.^3.*P3_target(2);
    x_bezier = x_bezier'; y_bezier = y_bezier';
    if nargin > 7 && isgraphics(ax_to_plot_on) && ~isempty(ax_to_plot_on) && any(ishandle(ax_to_plot_on))
        current_plots_in_ax = findobj(ax_to_plot_on); debug_tags = {'dbg_P0','dbg_P1','dbg_P2','dbg_P3','dbg_L01','dbg_L12','dbg_L23'};
        for k_tg = 1:length(debug_tags), delete(findobj(current_plots_in_ax, 'Tag', debug_tags{k_tg})); end
        hold(ax_to_plot_on, 'on');
        h_debug_plots(1)=plot(ax_to_plot_on,P0_actual(1),P0_actual(2),'ko','MarkerSize',6,'MarkerFaceColor','k','DisplayName','P0 (Bz Start)','Tag','dbg_P0');
        h_debug_plots(2)=plot(ax_to_plot_on,P1(1),P1(2),'gx','MarkerSize',10,'LineWidth',2,'DisplayName','P1 (Bz Ctrl)','Tag','dbg_P1');
        h_debug_plots(3)=plot(ax_to_plot_on,P2(1),P2(2),'bx','MarkerSize',10,'LineWidth',2,'DisplayName','P2 (Bz Ctrl)','Tag','dbg_P2');
        h_debug_plots(4)=plot(ax_to_plot_on,P3_target(1),P3_target(2),'ro','MarkerSize',6,'MarkerFaceColor','r','DisplayName','P3 (Bz End)','Tag','dbg_P3');
        h_debug_plots(5)=plot(ax_to_plot_on,[P0_actual(1) P1(1)],[P0_actual(2) P1(2)],'g:','HandleVisibility','off','Tag','dbg_L01');
        h_debug_plots(6)=plot(ax_to_plot_on,[P1(1) P2(1)],[P1(2) P2(2)],'k:','HandleVisibility','off','Tag','dbg_L12');
        h_debug_plots(7)=plot(ax_to_plot_on,[P2(1) P3_target(1)],[P2(2) P3_target(2)],'b:','HandleVisibility','off','Tag','dbg_L23');
        fprintf('DEBUG Bezier CPs Plotted: P0(%.1f,%.1f) P1(%.1f,%.1f) P2(%.1f,%.1f) P3(%.1f,%.1f)\n', P0_actual, P1, P2, P3_target);
    end
end

function g = geom(), g.Ltr = 2.20; g.Wtr = 1.00; g.Ltl = 1.0; g.Wtl = 1.00; g.Lh = 1.00; end % Ltl and Lh changed from your previous file values

function [prm_nodes, prm_adjacency] = buildPRMRoadmap(planning_map, base_map_for_occ_check, N_nodes, connection_radius_meters)
    fprintf('Step 1: Sampling %d free PRM nodes...\n', N_nodes);
    xLimits = planning_map.XWorldLimits; yLimits = planning_map.YWorldLimits;
    prm_nodes = []; attempts = 0; maxTries = N_nodes * 20; 
    map_cell_size = 1 / planning_map.Resolution; % Corrected
    while size(prm_nodes,1) < N_nodes && attempts < maxTries
        x = rand()*(xLimits(2)-xLimits(1))+xLimits(1); y = rand()*(yLimits(2)-yLimits(1))+yLimits(1); attempts=attempts+1;
        if getOccupancy(planning_map,[x,y])==0 && getOccupancy(base_map_for_occ_check,[x,y])==0, prm_nodes(end+1,:)=[x,y]; end
    end
    if size(prm_nodes,1)<N_nodes,warning('Only %d PRM nodes found.',size(prm_nodes,1));end; if size(prm_nodes,1)<2,prm_nodes=[];prm_adjacency=[];fprintf('Insufficient PRM nodes.\n');return;end
    fprintf(' Found %d PRM nodes.\n', size(prm_nodes,1));
    numActualNodes = size(prm_nodes,1); prm_adjacency = zeros(numActualNodes);
    for i=1:numActualNodes,for j=i+1:numActualNodes,p1=prm_nodes(i,:);p2=prm_nodes(j,:);dist=norm(p1-p2);
        if dist<=connection_radius_meters,num_checks=ceil(dist/(map_cell_size*0.8));num_checks=max(2,num_checks);
            ix=linspace(p1(1),p2(1),num_checks);iy=linspace(p1(2),p2(2),num_checks);is_ecf=true;
            for kc=1:num_checks,if getOccupancy(planning_map,[ix(kc),iy(kc)])==1,is_ecf=false;break;end;end
            if is_ecf,prm_adjacency(i,j)=dist;prm_adjacency(j,i)=dist;end;end;end;end
    fprintf(' PRM roadmap construction complete.\n');
end

function [path_coords, success] = findPathPRM(start_coords_xy, goal_coords_xy, existing_nodes, existing_adjacency, planning_map)
    path_coords = []; success = false; map_cell_size = 1 / planning_map.Resolution;
    if getOccupancy(planning_map,start_coords_xy)==1,fprintf('PRM Err: Start in obstacle!\n');return;end; if getOccupancy(planning_map,goal_coords_xy)==1,fprintf('PRM Err: Goal in obstacle!\n');return;end
    curr_nodes=[start_coords_xy;goal_coords_xy;existing_nodes];numCurrNodes=size(curr_nodes,1);curr_adj=zeros(numCurrNodes);
    if ~isempty(existing_adjacency)&&size(existing_adjacency,1)==size(existing_nodes,1)&&size(existing_adjacency,2)==size(existing_nodes,1),curr_adj(3:end,3:end)=existing_adjacency;elseif ~isempty(existing_adjacency),warning('PRM Adj matrix mismatch.');end
    conn_rad_sg=25;
    for i=3:numCurrNodes,for j=1:2,dist=norm(curr_nodes(i,:)-curr_nodes(j,:));
        if dist<=conn_rad_sg,num_c=ceil(dist/(map_cell_size*0.8));num_c=max(2,num_c);ix=linspace(curr_nodes(i,1),curr_nodes(j,1),num_c);iy=linspace(curr_nodes(i,2),curr_nodes(j,2),num_c);
        is_ecf=true;for kc=1:num_c,if getOccupancy(planning_map,[ix(kc),iy(kc)])==1,is_ecf=false;break;end;end;if is_ecf,curr_adj(i,j)=dist;curr_adj(j,i)=dist;end;end;end;end
    dist_sg=norm(start_coords_xy-goal_coords_xy);
    if dist_sg<=conn_rad_sg*2,num_c=ceil(dist_sg/(map_cell_size*0.8));num_c=max(2,num_c);ix=linspace(start_coords_xy(1),goal_coords_xy(1),num_c);iy=linspace(start_coords_xy(2),goal_coords_xy(2),num_c);
    is_ecf=true;for kc=1:num_c,if getOccupancy(planning_map,[ix(kc),iy(kc)])==1,is_ecf=false;break;end;end;if is_ecf,curr_adj(1,2)=dist_sg;curr_adj(2,1)=dist_sg;end;end
    sIdx=1;gIdx=2;visited=false(1,numCurrNodes);dist_node=inf(1,numCurrNodes);prev_node=nan(1,numCurrNodes);dist_node(sIdx)=0;
    for kd=1:numCurrNodes,tmp_dist=dist_node;tmp_dist(visited)=inf;[min_v,u]=min(tmp_dist);if isinf(min_v)||u==0,fprintf('PRM Dijkstra: No path (unreachable).\n');return;end;visited(u)=true;if u==gIdx,break;end
    neigh_idx=find(curr_adj(u,:)>0);for v=neigh_idx,if ~visited(v),alt_d=dist_node(u)+curr_adj(u,v);if alt_d<dist_node(v),dist_node(v)=alt_d;prev_node(v)=u;end;end;end;end
    if isinf(dist_node(gIdx)),fprintf('PRM Dijkstra: No path to goal.\n');return;end
    path_idx_rec=gIdx;if all(isnan(prev_node))&&gIdx~=sIdx,fprintf('PRM Path Rec Err: prev_node all NaNs.\n');return;end
    while ~isnan(prev_node(path_idx_rec(1)))&&prev_node(path_idx_rec(1))~=0&&prev_node(path_idx_rec(1))~=sIdx,path_idx_rec=[prev_node(path_idx_rec(1)),path_idx_rec];if length(path_idx_rec)>numCurrNodes*2,warning('PRM Path rec loop?');path_coords=[];success=false;return;end;end
    if ~isempty(path_idx_rec)&&~isnan(prev_node(path_idx_rec(1)))&&prev_node(path_idx_rec(1))==sIdx,path_idx_rec=[sIdx,path_idx_rec];path_coords=curr_nodes(path_idx_rec,:);success=true;fprintf('PRM Path found: %d segments.\n',length(path_idx_rec)-1);
    else,fprintf('PRM Path rec failed/start=goal.\n');if sIdx==gIdx,path_coords=curr_nodes(sIdx,:);success=true;else,path_coords=[];success=false;end;end
end
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
function animate_vehicles_with_map(map, x_ref, y_ref, truck, trailer, target_xs,target_ys)
    if isempty(truck) || ~isfield(truck,'x') || isempty(truck.x), fprintf('Animation skipped: Truck sim data empty.\n'); return; end
    if ~isempty(target_xs) && isrow(target_xs), target_xs = target_xs'; end; if ~isempty(target_ys) && isrow(target_ys), target_ys = target_ys'; end
    fig_name_anim = 'Truck-Trailer on Open Field'; 
    existing_fig = findall(0,'Type','figure','Name',fig_name_anim);
    if ~isempty(existing_fig), fig_anim = existing_fig(1); figure(fig_anim); ax_anim = fig_anim.CurrentAxes; cla(ax_anim); 
    else, fig_anim = figure('Name',fig_name_anim, 'Color','white', 'Position', [1000 50 800 700]); ax_anim = axes(fig_anim); end
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

function rect = get_rectangle_for_anim(x_center, y_center, theta_rad, L, W)
    R_matrix = [cos(theta_rad) -sin(theta_rad); sin(theta_rad)  cos(theta_rad)];
    corners_local = [-L/2 -W/2; L/2 -W/2; L/2 W/2; -L/2 W/2]; 
    rect = (R_matrix * corners_local')' + repmat([x_center y_center],4,1);
end