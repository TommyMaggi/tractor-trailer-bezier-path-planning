% run_method_B_bounding_circle.m - Method B: risk-based hazard detection
% via a bounding circle (proximity) combined with the hitch angle (articulation).
% This script orchestrates PRM path planning, path smoothing,
% curve detection, and curve modification for a truck-trailer system.
% Requires the simulate_vehicle.m and animate_vehicles_with_map.m in this
% folder: they compute and draw the bounding circle.

clear; clc;
close all;

% --- Step 1-5: Map Generation, PRM, Dijkstra, and Path Smoothing ---
fprintf('--- Steps 1-5: Generating Smoothed Reference Path ---\n');
topView = imread('Obstacles.png'); 
imageMask = createImageMask(topView);
imageMaskResized = imresize(imageMask, [333,500]);
openField = binaryOccupancyMap(50,33.3,10);
setOccupancy(openField, [1 1], ~imageMaskResized, "grid");
planningMap = copy(openField); 
inflateRadius = 1;
inflate(planningMap, inflateRadius);

N = 250;
maxTries = 10000;
connectionRadius = 5;

xLimits = openField.XWorldLimits; 
yLimits = openField.YWorldLimits;
nodes = [];
attempts = 0;
while size(nodes,1) < N && attempts < maxTries
    x = rand() * (xLimits(2) - xLimits(1)) + xLimits(1); 
    y = rand() * (yLimits(2) - yLimits(1)) + yLimits(1);
    attempts = attempts + 1;
    if getOccupancy(planningMap, [x,y]) == 0, nodes(end+1, :) = [x,y]; end
end
if size(nodes,1) < N, warning('Only %d free nodes found.', size(nodes,1)); end

numNodes = size(nodes, 1);
adjacency = zeros(numNodes);
for i = 1:numNodes 
    for j = i+1:numNodes
        p1 = nodes(i,:); p2 = nodes(j,:); dist = norm(p1-p2);
        if dist <= connectionRadius && isCollisionFree(planningMap, p1, p2)
            adjacency(i,j) = dist; adjacency(j,i) = dist;
        end
    end
end

start_node_coords = [5,5]; 
goal_node_coords = [43,30];
original_numNodes = numNodes;
nodes = [start_node_coords; goal_node_coords; nodes]; 
numNodes = size(nodes,1);
new_adjacency = zeros(numNodes);
if original_numNodes > 0, new_adjacency(3:end, 3:end) = adjacency; end
adjacency = new_adjacency; 
for i = 3:numNodes 
    for j = 1:2
        dist = norm(nodes(i,:) - nodes(j,:));
        if dist <= connectionRadius && isCollisionFree(planningMap, nodes(i,:), nodes(j,:))
            adjacency(i,j) = dist; adjacency(j,i) = dist;
        end
    end
end

startIdx = 1; goalIdx = 2; visited = false(1, numNodes);
dist_to_node = inf(1, numNodes); prev_node = nan(1, numNodes);
dist_to_node(startIdx) = 0;
for k_dijkstra = 1:numNodes
    temp_dist = dist_to_node; temp_dist(visited) = inf;
    [min_val, u] = min(temp_dist);
    if isinf(min_val), break; end
    visited(u) = true;
    if u == goalIdx, break; end
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
if isinf(dist_to_node(goalIdx)), error('No path found.'); end
path_indices = goalIdx; 
while prev_node(path_indices(1)) ~= startIdx && ~isnan(prev_node(path_indices(1)))
    path_indices = [prev_node(path_indices(1)), path_indices];
end
path_indices = [startIdx, path_indices];

x_ref_prm = nodes(path_indices,1); 
y_ref_prm = nodes(path_indices,2);
cum_dist_prm = [0; cumsum(vecnorm(diff([x_ref_prm, y_ref_prm]),2,2))];
num_interp_points = 2200;
interp_dist_points = linspace(0, cum_dist_prm(end), num_interp_points);
x_ref = interp1(cum_dist_prm, x_ref_prm, interp_dist_points, 'pchip'); 
y_ref = interp1(cum_dist_prm, y_ref_prm, interp_dist_points, 'pchip');
x_ref = smoothdata(smoothdata(smoothdata(x_ref, 'gaussian', 60), 'gaussian', 45), 'gaussian', 25);
y_ref = smoothdata(smoothdata(smoothdata(y_ref, 'gaussian', 60), 'gaussian', 45), 'gaussian', 25);

dx_for_theta = diff(x_ref); 
dy_for_theta = diff(y_ref);
theta_ref_calc = atan2(dy_for_theta, dx_for_theta);
theta_ref_calc = smoothdata(unwrap(theta_ref_calc), 'gaussian', 10);
theta_ref = [theta_ref_calc(:); theta_ref_calc(end)]; 
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
fprintf('--- Finished Steps 1-5 ---\n\n');
%%
%%% REPLACED SECTION: "Bounding Circle + Hitch Angle" Logic %%%
% --- Step 6: Detect Dangerous Segments via Bounding Circle ---
fprintf('--- Step 6: Detecting Segments with Bounding Circle + Hitch Angle Method ---\n');

% --- Parameters for this detection method ---
g = geom();
g.bounding_radius = (g.Ltr + g.Lh + g.Ltl) / 2; % Corrected radius
hitch_angle_threshold = deg2rad(30);
fprintf('  Parameters -> Bounding Radius: %.2f m, Hitch Angle Threshold: %.2f deg\n', g.bounding_radius, rad2deg(hitch_angle_threshold));

% --- 6.1: Run a preliminary simulation to get all necessary data ---
fprintf('  6.1: Running preliminary simulation for detection...\n');
[truck_sim_orig, trailer_sim_orig, target_xs_orig, target_ys_orig, bounding_circle_data_orig] = ...
    simulate_vehicle(x_ref, y_ref, openField, g);
num_sim_steps = length(trailer_sim_orig.x);
fprintf('    Simulation complete with %d steps.\n', num_sim_steps);

% --- 6.2: Check Proximity and Articulation to find dangerous moments ---
fprintf('  6.2: Checking for proximity and high articulation...\n');
is_dangerous = false(num_sim_steps, 1);
for i = 1:num_sim_steps
    proximity_alert = bounding_circle_data_orig.is_intersecting(i);
    hitch_angle = abs(angdiff(truck_sim_orig.theta(i), trailer_sim_orig.theta(i)));
    articulation_alert = hitch_angle > hitch_angle_threshold;
    if proximity_alert && articulation_alert
        is_dangerous(i) = true;
    end
end
fprintf('    Proximity and articulation checks complete.\n');

% --- 6.3: Apply "Maneuver Window" to find path segments ---
fprintf('  6.3: Defining Maneuver Window to identify correct path segments...\n');
causal_distance = g.Lh + g.Ltl;
path_distances = vecnorm(diff([x_ref(:), y_ref(:)]), 2, 2);
avg_step_dist = mean(path_distances);
index_shift = round(causal_distance / avg_step_dist);
padding = 35;
merge_index_gap_threshold = 50;
fprintf('    Causal distance: %.2f m -> Index Shift: %d points\n', causal_distance, index_shift);

raw_danger_bounds = []; in_danger = false;
for i = 1:length(is_dangerous)
    if is_dangerous(i) && ~in_danger
        start_idx = i; in_danger = true;
    elseif (~is_dangerous(i) || i == length(is_dangerous)) && in_danger
        end_idx = i - 1 + (i == length(is_dangerous) && is_dangerous(i));
        in_danger = false;
        raw_danger_bounds = [raw_danger_bounds; start_idx, end_idx];
    end
end

maneuver_bounds = [];
if ~isempty(raw_danger_bounds)
    maneuver_bounds(:,1) = max(1, raw_danger_bounds(:,1) - index_shift);
    maneuver_bounds(:,2) = raw_danger_bounds(:,2);
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
    if i > 1 && ~isempty(curve_bounds_padded), start_idx_padded = max(start_idx_padded, curve_bounds_padded(end, 2) + 1); end
    end_idx_padded = min(length(x_ref), e + padding);
    if i < size(merged_bounds, 1), end_idx_padded = min(end_idx_padded, merged_bounds(i+1, 1) - 1); end
    if start_idx_padded < end_idx_padded, curve_bounds_padded = [curve_bounds_padded; start_idx_padded, end_idx_padded]; end
end
fprintf('  Detected %d final maneuver window(s) for modification.\n', size(curve_bounds_padded,1));
fprintf('--- Finished Step 6 ---\n\n');
%%% END: REPLACED SECTION %%%

% --- Initial Simulation (using data from Step 6)---
fprintf('Step 7: Visualizing original smoothed path...\n');
% The simulation data (truck_sim_orig, etc.) was already generated in Step 6.1
animate_vehicles_with_map(openField, x_ref, y_ref, truck_sim_orig, trailer_sim_orig, target_xs_orig, target_ys_orig, bounding_circle_data_orig, is_dangerous);
fprintf(' Simulation visualization complete.\n');

% --- Centralized Vehicle Definitions and Costmaps ---
fprintf('Step 7.1: Setting up vehicle definitions and costmaps...\n');
current_geom = g;
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
costmap_truck_close = []; costmap_trailer_close = [];
if CLOSE_CALL_THRESHOLD_GLOBAL > 0
    inflated_map_for_close_calls = copy(openField);
    inflate(inflated_map_for_close_calls, CLOSE_CALL_THRESHOLD_GLOBAL);
    costmap_truck_close = vehicleCostmap(inflated_map_for_close_calls, 'CollisionChecker', truck_checker_hard);
    costmap_trailer_close = vehicleCostmap(inflated_map_for_close_calls, 'CollisionChecker', trailer_checker_hard);
end
app_costmaps = struct();
app_costmaps.truck_hard = costmap_truck_hard; app_costmaps.trailer_hard = costmap_trailer_hard;
app_costmaps.truck_close = costmap_truck_close; app_costmaps.trailer_close = costmap_trailer_close;
fprintf(' Vehicle definitions and costmaps created.\n');

% --- Global Hotspot Collection (Original Smoothed Path) ---
fprintf('Step 7.2: Collecting global collision hotspots from initial simulation...\n');
global_collision_hotspots = ...
    collect_global_hotspots_vehiclecostmap(truck_sim_orig, trailer_sim_orig, x_ref, y_ref, ...
                                           openField, current_geom, CLOSE_CALL_THRESHOLD_GLOBAL);
if ~isempty(global_collision_hotspots)
    fprintf(' Found %d potential problem spots (hard collisions or close calls).\n', length(global_collision_hotspots));
else
    fprintf(' No collision hotspots found in the initial global simulation.\n');
end

% --- Path Modification ---
fprintf('Step 8: Modifying curves with Recker-Bézier approach...\n');
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
    fprintf('\n!!! CRITICAL: Path modification failed.\n');
else
    fprintf('\nPath modification completed.\n');
end

% --- Simulation of Modified Path ---
fprintf('Step 9: Simulating modified path...\n');
[truck_mod, trailer_mod, target_xs_mod, target_ys_mod, bounding_circle_data_mod] = ...
    simulate_vehicle(x_ref_mod, y_ref_mod, openField, g);
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

% --- Final Animation ---
fprintf('Step 11: Animating final path...\n');
% Calculate the 'is_dangerous' flags for the modified path animation
is_dangerous_mod = false(length(truck_mod.x), 1);
for i = 1:length(truck_mod.x)
    prox_alert = bounding_circle_data_mod.is_intersecting(i);
    hitch_alert = abs(angdiff(truck_mod.theta(i), trailer_mod.theta(i))) > hitch_angle_threshold;
    if prox_alert && hitch_alert, is_dangerous_mod(i) = true; end
end
animate_vehicles_with_map(openField, x_ref_mod, y_ref_mod, truck_mod, trailer_mod, target_xs_mod, target_ys_mod, bounding_circle_data_mod, is_dangerous_mod);
fprintf('--- Script Finished ---\n');

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


% --- Local helper functions ---
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
function mask = createImageMask(X)
    mask = X(:,:,1) > 100;
end
function isFree = isCollisionFree(map, p1, p2)
    isFree = all(checkOccupancy(map, [p1;p2]) == 0);
    if ~isFree, return; end
    num_checks = 10;
    interp_points = [linspace(p1(1),p2(1),num_checks)', linspace(p1(2),p2(2),num_checks)'];
    isFree = all(checkOccupancy(map, interp_points) == 0);
end