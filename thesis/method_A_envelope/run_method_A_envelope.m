% run_method_A_envelope.m - Method A: collision-based hazard detection
% via the precise swept-path envelope of the trailer.
% This script orchestrates PRM path planning, path smoothing,
% curve detection, and curve modification for a truck-trailer system.
% v2.4.0: Integrated global hotspot collection using vehicleCostmap.
% Passes hotspots to modify_curves_bezier.
% 'g' struct passed to modify_curves_bezier.
% MODIFIED: Integrated advanced envelope-based curve detection in Step 6.

clear; clc;
close all;

% --- Steps 1-5: Map, PRM, Dijkstra, Smoothing ---
% Steps 1-5 build the occupancy map, plan with PRM + Dijkstra and smooth the path.
% Load the image and create the occupancy map
topView = imread('Obstacles.png'); 
imageMask = createImageMask(topView);
imageMaskResized = imresize(imageMask, [333,500]);
openField = binaryOccupancyMap(50,33.3,10);
setOccupancy(openField, [1 1], ~imageMaskResized, "grid");
planningMap = copy(openField); 
    inflateRadius = 1; % meters, buffer for PRM planning
inflate(planningMap, inflateRadius);
figure;
show(planningMap); hold on;
title('Inflated Map for PRM Planning'); hold off; 
% PRM Parameters
N = 250;%250; % number of nodes 
maxTries = 10000; % max attempts to find N free nodes
connectionRadius = 5; % meters, for connecting PRM nodes
% Node Sampling
xLimits = openField.XWorldLimits; 
yLimits = openField.YWorldLimits;
nodes = [];
attempts = 0;
fprintf('Step 1: Sampling %d free nodes...\n', N); 
while size(nodes,1) < N && attempts < maxTries
    x = rand() * (xLimits(2) - xLimits(1)) + xLimits(1); 
    y = rand() * (yLimits(2) - yLimits(1)) + yLimits(1);
    attempts = attempts + 1;
    if getOccupancy(planningMap, [x,y]) == 0 
        nodes(end+1, :) = [x,y];
    end
end
if size(nodes,1) < N
    warning('Only %d free nodes found after %d attempts. Continuing with fewer nodes.', size(nodes,1), attempts); 
    if size(nodes,1) < 2
        error('Insufficient nodes found for PRM. Adjust parameters or map.');
    end
end
fprintf(' Found %d nodes after %d attempts.\n', size(nodes,1), attempts); 
% Visualization: Sampled Nodes
figure;
show(planningMap); hold on;
plot(nodes(:,1), nodes(:,2), 'b.', 'MarkerSize', 10); 
title('Step 1: Sampled Free Nodes');
xlabel('X (m)'); ylabel('Y (m)');
axis equal;
% --- PRM Roadmap Construction ---
fprintf('Step 2: Constructing PRM roadmap...\n'); 
numNodes = size(nodes, 1);
adjacency = zeros(numNodes);
for i = 1:numNodes 
    for j = i+1:numNodes
        p1 = nodes(i,:);
        p2 = nodes(j,:);
        dist = norm(p1-p2);
        if dist <= connectionRadius 
            if isCollisionFree(planningMap, p1, p2) % local helper, end of file
                adjacency(i,j) = dist; 
                adjacency(j,i) = dist;
                plot([p1(1), p2(1)], [p1(2), p2(2)], 'g-', 'LineWidth', 0.5);
            end
        end
    end
end
title('Step 2: PRM Roadmap Connections');
fprintf(' Roadmap construction complete.\n'); 
% Add Start and Goal Nodes
fprintf('Step 3: Adding start and goal nodes...\n');
start_node_coords = [5,5];%[5,5]; 
goal_node_coords = [45,30];%[45,30];
if getOccupancy(openField, start_node_coords) == 1 
    error('Start node is in an obstacle on the base map!');
end
if getOccupancy(openField, goal_node_coords) == 1 
    error('Goal node is in an obstacle on the base map!');
end
original_numNodes = numNodes;
nodes = [start_node_coords; goal_node_coords; nodes]; 
numNodes = size(nodes,1);
new_adjacency = zeros(numNodes);
if original_numNodes > 0
    new_adjacency(3:end, 3:end) = adjacency;
end
adjacency = new_adjacency; 
for i = 3:numNodes 
    for j = 1:2 % Connect to Start (1) and Goal (2)
        dist = norm(nodes(i,:) - nodes(j,:));
        if dist <= connectionRadius
            if isCollisionFree(planningMap, nodes(i,:), nodes(j,:))
                adjacency(i,j) = dist; 
                adjacency(j,i) = dist;
                plot([nodes(i,1), nodes(j,1)], [nodes(i,2), nodes(j,2)], 'm--', 'LineWidth', 0.7); 
            end
        end
    end
end
plot(start_node_coords(1), start_node_coords(2), 'ro', 'MarkerFaceColor','r', 'MarkerSize',8, 'DisplayName','Start');
plot(goal_node_coords(1), goal_node_coords(2), 'bs', 'MarkerFaceColor','b', 'MarkerSize',8, 'DisplayName','Goal'); 
fprintf(' Start and goal nodes added and connected.\n');
% --- Dijkstra's Algorithm for Path Finding ---
fprintf('Step 4: Finding path with Dijkstra...\n');
startIdx = 1;
goalIdx = 2;
visited = false(1, numNodes);
dist_to_node = inf(1, numNodes);
prev_node = nan(1, numNodes);
dist_to_node(startIdx) = 0;
for k_dijkstra = 1:numNodes
    temp_dist = dist_to_node;
    temp_dist(visited) = inf; % Consider only unvisited nodes
    [min_val, u] = min(temp_dist);
    if isinf(min_val) % No path found
        break;
    end
    visited(u) = true;
    if u == goalIdx
        fprintf(' Goal reached by Dijkstra.\n');
        break; % Goal reached
    end
    neighbors = find(adjacency(u,:) > 0);
    for v_neighbor = neighbors
        if ~visited(v_neighbor)
            alt_dist = dist_to_node(u) + adjacency(u, v_neighbor); 
            if alt_dist < dist_to_node(v_neighbor)
                dist_to_node(v_neighbor) = alt_dist;
                prev_node(v_neighbor) = u;
            end
        end
    end 
end 
if isinf(dist_to_node(goalIdx))
    error('No path found from start to goal using Dijkstra.');
end
% Reconstruct path
path_indices = goalIdx; 
while prev_node(path_indices(1)) ~= startIdx && ~isnan(prev_node(path_indices(1)))
    path_indices = [prev_node(path_indices(1)), path_indices];
end
if prev_node(path_indices(1)) == startIdx
    path_indices = [startIdx, path_indices];
else
    warning('Path reconstruction might be incomplete.'); 
    if isempty(path_indices) || path_indices(1) ~= startIdx % Failsafe
        path_indices = [startIdx, path_indices];
    end
end
for i = 1:length(path_indices)-1
    p1 = nodes(path_indices(i),:); 
    p2 = nodes(path_indices(i+1),:);
    plot([p1(1),p2(1)], [p1(2),p2(2)], 'r', 'LineWidth', 2.5);
end
title('Final PRM Path with Dijkstra');
fprintf(' Path found with %d segments.\n', length(path_indices)-1); 
hold off;
% --- Path Densification and Smoothing ---
fprintf('Step 5: Densifying and smoothing path (AGRESSIVE SMOOTHING)...\n');
x_ref_prm = nodes(path_indices,1); 
y_ref_prm = nodes(path_indices,2);
original_path_segments = [x_ref_prm, y_ref_prm];
segment_lengths_prm = vecnorm(diff(original_path_segments,1,1),2,2); 
cum_dist_prm = [0; cumsum(segment_lengths_prm)]; % Ensure it starts at 0
num_interp_points = 2200; % Increased for potentially smoother curves before detection
interp_dist_points = linspace(0, cum_dist_prm(end), num_interp_points);
x_ref = interp1(cum_dist_prm, x_ref_prm, interp_dist_points, 'pchip'); 
y_ref = interp1(cum_dist_prm, y_ref_prm, interp_dist_points, 'pchip');
% AGGRESSIVE SMOOTHING
smoothing_window_pass1 = 60; 
smoothing_window_pass2 = 45;
smoothing_window_pass3 = 25;
fprintf(' Applying Gaussian smoothing pass 1 (window: %d)...\n', smoothing_window_pass1);
x_ref = smoothdata(x_ref, 'gaussian', smoothing_window_pass1); 
y_ref = smoothdata(y_ref, 'gaussian', smoothing_window_pass1);
fprintf(' Applying Gaussian smoothing pass 2 (window: %d)...\n', smoothing_window_pass2);
x_ref = smoothdata(x_ref, 'gaussian', smoothing_window_pass2); 
y_ref = smoothdata(y_ref, 'gaussian', smoothing_window_pass2);
fprintf(' Applying Gaussian smoothing pass 3 (window: %d)...\n', smoothing_window_pass3);
x_ref = smoothdata(x_ref, 'gaussian', smoothing_window_pass3); 
y_ref = smoothdata(y_ref, 'gaussian', smoothing_window_pass3);
fprintf(' Path densified to %d points and aggressively smoothed.\n', length(x_ref));
% Calculate the definitive theta_ref from the final smoothed path
dx_for_theta = diff(x_ref); 
dy_for_theta = diff(y_ref);
if isempty(dx_for_theta) || isempty(dy_for_theta) 
    error('Path too short after aggressive smoothing for final theta_ref calculation.');
end
theta_ref_calc = atan2(dy_for_theta, dx_for_theta);
theta_ref_calc = smoothdata(unwrap(theta_ref_calc), 'gaussian', 10); % Smooth theta
theta_ref = [theta_ref_calc(:); theta_ref_calc(end)]; % Ensure column and replicate last for length consistency 
fprintf(' Definitive theta_ref calculated and smoothed for subsequent steps.\n'); 

% --- Optional: reuse the exact reference path of the thesis run ---
% PRM samples random nodes, so every run produces a different reference path.
% Set USE_THESIS_PATH = true to replace it with the path used for the results
% reported in the thesis (same path for Method A and Method B).
USE_THESIS_PATH = false;
if USE_THESIS_PATH
    S_thesis = load(fullfile('..', 'results', 'method_A_workspace.mat'), 'x_ref', 'y_ref', 'theta_ref');
    x_ref = S_thesis.x_ref; y_ref = S_thesis.y_ref; theta_ref = S_thesis.theta_ref;
    fprintf(' Using the saved thesis reference path (%d points).\n', numel(x_ref));
end
%%
%%% START: Step 6 - Method A, envelope-based hazard detection %%%
% --- Step 6: Detect Dangerous Segments via Precise Swept Path ---
fprintf('Step 6: Detecting dangerous segments via Precise Swept Path method.\n');

% --- Parameters for Collision Segment Processing ---
merge_index_gap_threshold = 50;
padding = 35;

% --- 6.1: Run a preliminary simulation to get data for envelope---
fprintf('  6.1: Running preliminary simulation...\n');
g = geom(); % Use local geom() function
% Note: a separate `simulate_vehicle` function is required.
% We assume it's available on the path.
[truck_sim_for_detection, trailer_sim_for_detection, ~, ~] = simulate_vehicle(x_ref, y_ref);
num_sim_steps = length(trailer_sim_for_detection.x);
fprintf('    Simulation complete with %d steps.\n', num_sim_steps);

% --- 6.2: Build Precise Envelope with polyshape and Check Collisions ---
fprintf('  6.2: Building precise envelope and checking for collisions...\n');
local_corners = [ g.Ltl, -g.Wtl/2; g.Ltl, g.Wtl/2; 0, -g.Wtl/2; 0, g.Wtl/2 ]; % FR, FL, RR, RL

% Initialize polyshape and trajectory arrays
swept_path_envelope = polyshape(); 
is_colliding = false(num_sim_steps, 1);
FR_traj = zeros(num_sim_steps, 2); FL_traj = zeros(num_sim_steps, 2);
RR_traj = zeros(num_sim_steps, 2); RL_traj = zeros(num_sim_steps, 2);

for i = 1:num_sim_steps
    x_t = trailer_sim_for_detection.x(i); y_t = trailer_sim_for_detection.y(i); theta_t = trailer_sim_for_detection.theta(i);
    R = [cos(theta_t), -sin(theta_t); sin(theta_t),  cos(theta_t)];
    world_corners = (R * local_corners' + [x_t; y_t])';
    
    FR_traj(i, :) = world_corners(1,:); FL_traj(i, :) = world_corners(2,:);
    RR_traj(i, :) = world_corners(3,:); RL_traj(i, :) = world_corners(4,:);

    current_patch = polyshape(world_corners([3,1,2,4],:));
    if mod(i, 5) == 0 || i == num_sim_steps
        swept_path_envelope = union(swept_path_envelope, current_patch);
    end

    for j = 1:4
        if getOccupancy(openField, world_corners(j,:)) == 1
            is_colliding(i) = true;
            break; 
        end
    end
end
swept_path_envelope = rmholes(swept_path_envelope);
fprintf('    Precise envelope built and collision check complete.\n');

% --- 6.3: Identify, Merge, and Pad Collision Segments using Maneuver Window ---
fprintf('  6.3: Defining Maneuver Window to identify correct path segments...\n');

causal_distance = g.Lh + g.Ltl+ 2*g.Ltr; 
path_distances = vecnorm(diff([x_ref(:), y_ref(:)]), 2, 2);
avg_step_dist = mean(path_distances);
index_shift = round(causal_distance / avg_step_dist);
fprintf('    Causal distance: %.2f m -> Index Shift: %d points\n', causal_distance, index_shift);

raw_collision_bounds = [];
in_collision = false;
for i = 1:length(is_colliding)
    if is_colliding(i) && ~in_collision
        start_idx = i;
        in_collision = true;
    elseif (~is_colliding(i) || i == length(is_colliding)) && in_collision
        end_idx = i - 1 + (i == length(is_colliding) && is_colliding(i));
        in_collision = false;
        raw_collision_bounds = [raw_collision_bounds; start_idx, end_idx];
    end
end

maneuver_bounds = [];
if ~isempty(raw_collision_bounds)
    maneuver_bounds(:,1) = max(1, raw_collision_bounds(:,1) - index_shift);
    maneuver_bounds(:,2) = raw_collision_bounds(:,2);
end

merged_bounds = [];
if ~isempty(maneuver_bounds)
    current_bound = maneuver_bounds(1,:);
    for i = 2:size(maneuver_bounds, 1)
        next_bound = maneuver_bounds(i,:);
        if (next_bound(1) - current_bound(2)) <= merge_index_gap_threshold
            current_bound(2) = next_bound(2);
        else
            merged_bounds = [merged_bounds; current_bound];
            current_bound = next_bound;
        end
    end
    merged_bounds = [merged_bounds; current_bound];
end

curve_bounds_padded = [];
for i = 1:size(merged_bounds, 1)
    s = merged_bounds(i,1); e = merged_bounds(i,2);
    start_idx_padded = max(1, s - padding);
    if i > 1 && ~isempty(curve_bounds_padded)
        prev_end = curve_bounds_padded(end, 2);
        start_idx_padded = max(start_idx_padded, prev_end + 1);
    end
    end_idx_padded = min(length(x_ref), e + padding);
    if i < size(merged_bounds, 1)
        next_start = merged_bounds(i+1, 1);
        end_idx_padded = min(end_idx_padded, next_start - 1);
    end
    if start_idx_padded < end_idx_padded
        curve_bounds_padded = [curve_bounds_padded; start_idx_padded, end_idx_padded];
    end
end
fprintf('  Detected %d final maneuver window(s) for modification.\n', size(curve_bounds_padded,1));

% --- 6.4: Visualization - Polyshape + Corner Traces ---
fprintf('  6.4: Generating visualization...\n');
figure('Name', 'Precise Swept Path with Corner Traces');
show(openField); hold on;
plot(swept_path_envelope, 'FaceColor', 'green', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', 'Precise Swept Area');
plot(FL_traj(:,1), FL_traj(:,2), 'c-', 'LineWidth', 1, 'DisplayName', 'Front Corner Paths');
plot(FR_traj(:,1), FR_traj(:,2), 'c-', 'LineWidth', 1, 'HandleVisibility', 'off');
plot(RL_traj(:,1), RL_traj(:,2), 'm-', 'LineWidth', 1, 'DisplayName', 'Rear Corner Paths');
plot(RR_traj(:,1), RR_traj(:,2), 'm-', 'LineWidth', 1, 'HandleVisibility', 'off');
plot(x_ref, y_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Path');
if ~isempty(curve_bounds_padded)
    for i = 1:size(curve_bounds_padded,1)
        s_idx = curve_bounds_padded(i,1); e_idx = curve_bounds_padded(i,2);
        plot(x_ref(s_idx:e_idx), y_ref(s_idx:e_idx), 'r-', 'LineWidth', 2.5, 'DisplayName', sprintf('Segment %d to Modify', i));
    end
end
title('Precise Swept Path Envelope with Corner Traces');
xlabel('X (m)'); ylabel('Y (m)'); legend('show', 'Location', 'best'); axis equal; hold off;
%%% END: New Step 6 %%%

% --- Initial Simulation (Original Smoothed Path) ---
% Note: The simulation for detection in Step 6.1 gives us the data we need.
% We can reuse it here to avoid running the same simulation twice.
fprintf('Step 7: Simulating original smoothed path...\n');
truck_sim_orig = truck_sim_for_detection;
trailer_sim_orig = trailer_sim_for_detection;
% We still need to run the full simulation to get target_xs_orig, etc., if needed by animate
[~, ~, target_xs_orig, target_ys_orig] = simulate_vehicle(x_ref, y_ref);
fprintf(' Simulation complete.\n');

% --- Visualize Initial Simulation ---
figure('Name', 'Initial Simulation on Smoothed Path'); 
animate_vehicles_with_map(openField, x_ref, y_ref, truck_sim_orig, trailer_sim_orig, target_xs_orig, target_ys_orig); hold on;
plot(swept_path_envelope, 'FaceColor', 'green', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', 'Precise Swept Area');

% --- Centralized Vehicle Definitions and Costmaps ---
fprintf('Step 7.1: Setting up vehicle definitions and costmaps...\n');
current_geom = geom(); 
RO_truck_param = current_geom.Ltr * 0.15;
WB_truck_param = current_geom.Ltr * 0.70;
FO_truck_param = current_geom.Ltr - WB_truck_param - RO_truck_param;
if FO_truck_param < -1e-6, FO_truck_param = max(0, FO_truck_param); end
truck_dimensions = vehicleDimensions(current_geom.Ltr, current_geom.Wtr, current_geom.Wtr, ...
                                 'Wheelbase', WB_truck_param, ...
                                 'RearOverhang', RO_truck_param, ...
                                 'FrontOverhang', FO_truck_param);
truck_checker_hard = inflationCollisionChecker(truck_dimensions, 'NumCircles', 5);
RO_trailer_param = current_geom.Ltl * 0.15;
WB_trailer_param = current_geom.Ltl * 0.70;
FO_trailer_param = current_geom.Ltl - WB_trailer_param - RO_trailer_param;
if FO_trailer_param < -1e-6, FO_trailer_param = max(0, FO_trailer_param); end
trailer_dimensions = vehicleDimensions(current_geom.Ltl, current_geom.Wtl, current_geom.Wtl, ...
                                   'Wheelbase', WB_trailer_param, ...
                                   'RearOverhang', RO_trailer_param, ...
                                   'FrontOverhang', FO_trailer_param);
trailer_checker_hard = inflationCollisionChecker(trailer_dimensions, 'NumCircles', 3);
current_geom.WB_truck = WB_truck_param;
current_geom.WB_trailer = WB_trailer_param;
costmap_truck_hard = vehicleCostmap(openField, 'CollisionChecker', truck_checker_hard);
costmap_trailer_hard = vehicleCostmap(openField, 'CollisionChecker', trailer_checker_hard);
CLOSE_CALL_THRESHOLD_GLOBAL = 0.30;
costmap_truck_close = [];
costmap_trailer_close = [];
if CLOSE_CALL_THRESHOLD_GLOBAL > 0
    fprintf(' Inflating map for close call detection (threshold: %.2fm).\n', CLOSE_CALL_THRESHOLD_GLOBAL);
    inflated_map_for_close_calls = copy(openField);
    inflate(inflated_map_for_close_calls, CLOSE_CALL_THRESHOLD_GLOBAL);
    costmap_truck_close = vehicleCostmap(inflated_map_for_close_calls, 'CollisionChecker', truck_checker_hard);
    costmap_trailer_close = vehicleCostmap(inflated_map_for_close_calls, 'CollisionChecker', trailer_checker_hard);
else
    fprintf(' Close call detection threshold is zero or less; close-call costmaps will not be used.\n');
end
app_costmaps = struct();
app_costmaps.truck_hard = costmap_truck_hard;
app_costmaps.trailer_hard = costmap_trailer_hard;
app_costmaps.truck_close = costmap_truck_close;
app_costmaps.trailer_close = costmap_trailer_close;
fprintf(' Vehicle definitions and costmaps created.\n');

% --- Global Hotspot Collection (Original Smoothed Path) ---
fprintf('Step 7.2: Collecting global collision hotspots from initial simulation...\n');
global_collision_hotspots = ...
    collect_global_hotspots_vehiclecostmap(truck_sim_orig, trailer_sim_orig, x_ref, y_ref, ...
                                           openField, current_geom, CLOSE_CALL_THRESHOLD_GLOBAL);
if ~isempty(global_collision_hotspots)
    fprintf(' Found %d potential problem spots (hard collisions or close calls) in the initial global simulation.\n', length(global_collision_hotspots));
    figure; show(openField); hold on; plot(x_ref,y_ref, 'k--'); axis equal;
    title('Global Collision Hotspots from Initial Simulation');
    for k_spot = 1:length(global_collision_hotspots)
        spot_pose = global_collision_hotspots(k_spot).vehicle_pose;
        spot_type = global_collision_hotspots(k_spot).vehicle_type;
        is_hard = global_collision_hotspots(k_spot).is_hard_collision;
        plot_color = 'yo'; 
        if is_hard
            plot_color = 'rx'; 
        end
        plot(spot_pose(1), spot_pose(2), plot_color, 'MarkerSize', 8, 'LineWidth', 1.5);
        text(spot_pose(1), spot_pose(2), sprintf('%s H%d', spot_type(1), k_spot), 'FontSize',8);
    end
    hold off;
else
    fprintf(' No collision hotspots found in the initial global simulation. Path appears safe.\n');
end

% --- Path Modification ---
fprintf('Step 8: Modifying curves with Recker-Bézier approach (informed by global hotspots)...\n');
[x_ref_mod, y_ref_mod, overall_modification_successful] = ...
    modify_curves_bezier(x_ref, y_ref, theta_ref, curve_bounds_padded, ...
                         openField, ...
                         truck_sim_orig, ...
                         trailer_sim_orig, ...
                         current_geom, ...
                         RO_truck_param, ...
                         RO_trailer_param, ...
                         app_costmaps, ...
                         truck_dimensions, ...
                         trailer_dimensions, ...
                         global_collision_hotspots);
if ~overall_modification_successful
    fprintf('\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n');
    fprintf('!!! CRITICAL: Path modification failed to produce a feasible path for all curves.\n');
    fprintf('!!! Path is UNFEASIBLE. Consider re-planning or adjusting parameters.\n');
    fprintf('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n');
else
    fprintf('\nPath modification completed. All segments reported as successfully processed (or did not require modification).\n');
end

% --- Simulation of Modified Path ---
fprintf('Step 9: Simulating modified path...\n');
[truck_mod, trailer_mod, target_xs_mod, target_ys_mod] = simulate_vehicle(x_ref_mod, y_ref_mod);
fprintf(' Simulation of modified path complete.\n');

% --- Final Visualization ---
fprintf('Step 10: Visualizing final results...\n');
figure('Name', 'Final Simulation Comparison');
show(openField); hold on;
plot(x_ref, y_ref, 'k--', 'DisplayName', 'Original Smoothed Path');
plot(x_ref_mod, y_ref_mod, 'b-', 'LineWidth',2, 'DisplayName', 'Modified Path');
if ~isempty(truck_mod.x)
    plot(truck_mod.x, truck_mod.y, 'c-', 'DisplayName', 'Truck on Modified Path');
    plot(trailer_mod.x, trailer_mod.y, 'r-', 'DisplayName', 'Trailer on Modified Path');
end
legend('Location','best');
axis equal;
title('Path Comparison: Original, Modified, and Simulated Vehicle');
hold off;
% Note: The animation call requires the `all_corner_points` variable.
% We must reconstruct it for the trail to work.
all_corner_points_for_trail = [FR_traj; FL_traj; RR_traj; RL_traj];
figure('Name', 'Final Animation with Trail');
show(openField); hold on;
plot(swept_path_envelope, 'FaceColor', 'green', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', 'Precise Swept Area');
plot(all_corner_points_for_trail(:,1), all_corner_points_for_trail(:,2), '.', 'Color', [0.8 0.8 0.8], 'DisplayName', 'Trailer Envelope Trail');
animate_vehicles_with_map(openField, x_ref_mod, y_ref_mod, truck_mod, trailer_mod, target_xs_mod, target_ys_mod);
fprintf('--- Script Finished ---\n');
figure;
%%
% %fprintf('Step 10: Visualizing final results...\n');
% figure('Name', 'Final Paths Comparison');
% show(openField); hold on;
% plot(x_ref, y_ref, 'k--', 'LineWidth',2, 'DisplayName', 'Original Smoothed Path');
% plot(x_ref_mod, y_ref_mod, 'b-', 'LineWidth',2, 'DisplayName', 'Modified Path');
% plot(x_mod_cir, y_mod_cir, 'r-', 'LineWidth',2, 'DisplayName', 'Modified Path');
% % if ~isempty(truck_mod.x)
% %     plot(truck_mod.x, truck_mod.y, 'c-', 'DisplayName', 'Truck on Modified Path');
% %     plot(trailer_mod.x, trailer_mod.y, 'r-', 'DisplayName', 'Trailer on Modified Path');
% % end
% legend('Location','best');
% axis equal;
%% --- Step 12: Analyze Metrics and Plot Clearance Over Time ---
fprintf('\n--- Analyzing Final Performance Metrics ---\n');
g = geom(); % Ensure the geometry struct is available

% --- Path Length Analysis ---
% Calculate the length of the original, unmodified path for comparison
original_path_points = [x_ref(:), y_ref(:)];
original_segment_lengths = vecnorm(diff(original_path_points, 1, 1), 2, 2);
original_path_length = sum(original_segment_lengths);

% Calculate the length of the final, modified path
modified_path_points = [x_ref_mod(:), y_ref_mod(:)];
modified_segment_lengths = vecnorm(diff(modified_path_points, 1, 1), 2, 2);
modified_path_length = sum(modified_segment_lengths);

fprintf('  Path Length Comparison:\n');
fprintf('    Original Path Length:  %.2f m\n', original_path_length);
fprintf('    Modified Path Length:  %.2f m\n', modified_path_length);
fprintf('    Path Length Increase:  %.2f %% \n', ((modified_path_length / original_path_length) - 1) * 100);

% --- Clearance Analysis ---
fprintf('  Analyzing Vehicle Clearance...\n');
% Call the analysis function with the final data from the modified path simulation
[truck_clearance, trailer_clearance] = analyze_clearance(truck_mod, trailer_mod, openField, g);

% Create a time vector for the x-axis of the plot
time_vector = (0:length(truck_clearance)-1) * g.dt;

% Create a new figure for the clearance plot
figure('Name', 'Vehicle Clearance Over Time');
plot(time_vector, truck_clearance, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Tractor Clearance');
hold on;
plot(time_vector, trailer_clearance, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Trailer Clearance');
hold off;

% Add plot details
grid on;
title('Minimum Vehicle Clearance from Obstacles');
xlabel('Time (s)');
ylabel('Clearance (m)');
legend('Location', 'best');
ylim([0, max([truck_clearance; trailer_clearance]) * 1.1]); % Set y-axis limit for clarity
fprintf('--- Metric analysis and plot generation complete ---\n');



% --- Helper Functions ---
function g = geom()
    g.Ltr  = 2.50;   
    g.Wtr  = 1.40;   
    g.Ltl  = 1.50;   
    g.Wtl  = 1.45;   
    g.Lh   = 2.00;   
    g.dt   = 0.05;   
    g.v    = 0.50;
    g.look = 1.00;   
end

% --- Local helpers (local functions take precedence over files with the same name) ---
function mask = createImageMask(X)
    % Binary mask: true = free (white) pixels
    mask = X(:,:,1) > 100;
end
function isFree = isCollisionFree(map, p1, p2)
    % Straight-line collision check between two points (10 samples)
    isFree = all(checkOccupancy(map, [p1;p2]) == 0);
    if ~isFree, return; end
    num_checks = 10;
    interp_points = [linspace(p1(1),p2(1),num_checks)', linspace(p1(2),p2(2),num_checks)'];
    isFree = all(checkOccupancy(map, interp_points) == 0);
end