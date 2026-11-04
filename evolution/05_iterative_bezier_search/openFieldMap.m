% Main script: openFieldMap.m
% This script orchestrates PRM path planning, path smoothing,
% curve detection, and curve modification for a truck-trailer system.
% v2.3.9: Unified theta_ref. Restored user's baseline curve detection parameters.
%         Refined spiky tail extension to be more targeted.
%         Adjusted merging thresholds to be less aggressive.
%         Ensured signal plots use consistent data.

%clear; clc; close all; % Keep clear/clc/close commented if running iteratively

% Load the image and create the occupancy map
topView = imread('Obstacles.png'); 
imageMask = createImageMask(topView); 
imageMaskResized = imresize(imageMask,[333,500]); 
openField = binaryOccupancyMap(50,33.3,10); 
setOccupancy(openField,[1 1],~imageMaskResized,"grid"); 

planningMap = copy(openField);
inflateRadius = 0.75; % meters, buffer for PRM planning
inflate(planningMap, inflateRadius);
figure;
show(planningMap); hold on;
title('Inflated Map for PRM Planning'); hold off;

% --- PRM Parameters ---
N = 250; % number of nodes
maxTries = 10000; % max attempts to find N free nodes
connectionRadius = 5; % meters, for connecting PRM nodes

% --- Node Sampling ---
xLimits = openField.XWorldLimits;
yLimits = openField.YWorldLimits;
nodes = [];
attempts = 0;
fprintf('Step 1: Sampling %d free nodes...\n', N);
while size(nodes, 1) < N && attempts < maxTries
    x = rand() * (xLimits(2) - xLimits(1)) + xLimits(1); 
    y = rand() * (yLimits(2) - yLimits(1)) + yLimits(1); 
    attempts = attempts + 1;
    if getOccupancy(planningMap, [x, y]) == 0 
        nodes(end+1, :) = [x, y];
    end
end
if size(nodes, 1) < N
    warning('Only %d free nodes found after %d attempts. Continuing with fewer nodes.', size(nodes,1), attempts);
    if size(nodes, 1) < 2
        error('Insufficient nodes found for PRM. Adjust parameters or map.');
    end
end
fprintf('  Found %d nodes after %d attempts.\n', size(nodes,1), attempts);

% --- Visualization: Sampled Nodes ---
figure;
show(planningMap); hold on;
plot(nodes(:,1), nodes(:,2), 'b.', 'MarkerSize', 10);
title('Step 1: Sampled Free Nodes');
xlabel('X (m)'); ylabel('Y (m)');
axis equal;

% --- PRM Roadmap Construction ---
fprintf('Step 2: Constructing PRM roadmap...\n');
numNodes = size(nodes,1);
adjacency = zeros(numNodes); 
for i = 1:numNodes
    for j = i+1:numNodes
        p1 = nodes(i, :);
        p2 = nodes(j, :);
        dist = norm(p1 - p2);
        if dist <= connectionRadius
            if isCollisionFree(planningMap, p1, p2) 
                adjacency(i, j) = dist;
                adjacency(j, i) = dist;
                plot([p1(1), p2(1)], [p1(2), p2(2)], 'g-', 'LineWidth', 0.5);
            end
        end
    end
end
title('Step 2: PRM Roadmap Connections');
fprintf('  Roadmap construction complete.\n');

% --- Add Start and Goal Nodes ---
fprintf('Step 3: Adding start and goal nodes...\n');
start_node_coords = [2.5, 2.5];     
goal_node_coords = [45, 30]; 

if getOccupancy(openField, start_node_coords) == 1
    error('Start node is in an obstacle on the base map!');
end
if getOccupancy(openField, goal_node_coords) == 1
    error('Goal node is in an obstacle on the base map!');
end

original_numNodes = numNodes;
nodes = [start_node_coords; goal_node_coords; nodes]; 
numNodes = size(nodes, 1);
new_adjacency = zeros(numNodes);
if original_numNodes > 0 
    new_adjacency(3:end, 3:end) = adjacency;
end
adjacency = new_adjacency;

for i = 3:numNodes 
    for j = 1:2 
        dist = norm(nodes(i,:) - nodes(j,:));
        if dist <= connectionRadius
            if isCollisionFree(planningMap, nodes(i,:), nodes(j,:))
                adjacency(i, j) = dist;
                adjacency(j, i) = dist;
                plot([nodes(i,1), nodes(j,1)], [nodes(i,2), nodes(j,2)], 'm--','LineWidth',0.7); 
            end
        end
    end
end
plot(start_node_coords(1), start_node_coords(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Start');
plot(goal_node_coords(1), goal_node_coords(2), 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Goal');
fprintf('  Start and goal nodes added and connected.\n');
%% 
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
    temp_dist(visited) = inf; 
    [min_val, u] = min(temp_dist); 
    
    if isinf(min_val) 
        break;
    end
    
    visited(u) = true;
    if u == goalIdx
        fprintf('  Goal reached by Dijkstra.\n');
        break; 
    end

    neighbors = find(adjacency(u, :) > 0);
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
path_indices = goalIdx;
while prev_node(path_indices(1)) ~= startIdx && ~isnan(prev_node(path_indices(1)))
    path_indices = [prev_node(path_indices(1)), path_indices];
end
if prev_node(path_indices(1)) == startIdx 
    path_indices = [startIdx, path_indices];
else 
    warning('Path reconstruction might be incomplete.');
    if isempty(path_indices) || path_indices(1) ~= startIdx 
        path_indices = [startIdx, path_indices]; 
    end
end

for i = 1:length(path_indices)-1
    p1 = nodes(path_indices(i), :);
    p2 = nodes(path_indices(i+1), :);
    plot([p1(1), p2(1)], [p1(2), p2(2)], 'r-', 'LineWidth', 2.5);
end
title('Final PRM Path with Dijkstra');
fprintf('  Path found with %d segments.\n', length(path_indices)-1);
hold off;
%% 
% --- Path Densification and Smoothing ---
fprintf('Step 5: Densifying and smoothing path (AGRESSIVE SMOOTHING)...\n');
x_ref_prm = nodes(path_indices, 1);
y_ref_prm = nodes(path_indices, 2);

original_path_segments = [x_ref_prm, y_ref_prm]; 
segment_lengths_prm = vecnorm(diff(original_path_segments,1,1), 2, 2);
cum_dist_prm = [0; cumsum(segment_lengths_prm)];

num_interp_points = 2500; 
interp_dist_points = linspace(0, cum_dist_prm(end), num_interp_points);
x_ref = interp1(cum_dist_prm, x_ref_prm, interp_dist_points, 'pchip');
y_ref = interp1(cum_dist_prm, y_ref_prm, interp_dist_points, 'pchip');

% AGGRESSIVE SMOOTHING
smoothing_window_pass1 = 60; 
smoothing_window_pass2 = 45; 
smoothing_window_pass3 = 25; 

fprintf('  Applying Gaussian smoothing pass 1 (window: %d)...\n', smoothing_window_pass1);
x_ref = smoothdata(x_ref, 'gaussian', smoothing_window_pass1);
y_ref = smoothdata(y_ref, 'gaussian', smoothing_window_pass1);
fprintf('  Applying Gaussian smoothing pass 2 (window: %d)...\n', smoothing_window_pass2);
x_ref = smoothdata(x_ref, 'gaussian', smoothing_window_pass2);
y_ref = smoothdata(y_ref, 'gaussian', smoothing_window_pass2);
fprintf('  Applying Gaussian smoothing pass 3 (window: %d)...\n', smoothing_window_pass3);
x_ref = smoothdata(x_ref, 'gaussian', smoothing_window_pass3);
y_ref = smoothdata(y_ref, 'gaussian', smoothing_window_pass3);

fprintf('  Path densified to %d points and aggressively smoothed.\n', length(x_ref));

% Calculate the definitive theta_ref from the final smoothed path
% This single theta_ref will be used by Step 6 and passed to Step 8.
dx_for_theta = diff(x_ref); % Use the final smoothed x_ref, y_ref
dy_for_theta = diff(y_ref);
if isempty(dx_for_theta) || isempty(dy_for_theta)
    error('Path too short after aggressive smoothing for final theta_ref calculation.');
end
theta_ref_calc = atan2(dy_for_theta, dx_for_theta);
theta_ref_calc = smoothdata(unwrap(theta_ref_calc), 'gaussian', 10); % Consistent with user's original theta smoothing for curve detection
theta_ref = [theta_ref_calc(:); theta_ref_calc(end)];
fprintf('  Definitive theta_ref calculated and smoothed for subsequent steps.\n');

%% 
%%% START MODIFICATION V2.3.9 - User's Baseline Curve Detection + Refined Spiky Tail Extension + Less Aggressive Merging
% --- Curve Detection Logic (User's baseline + Refined Spiky Tail Extension + Less Aggressive Merging) --- 
fprintf('Step 6: Detecting curves (User baseline + refined spiky tail extension + less aggressive merging).\n'); 
% x_ref, y_ref are from Step 5.
% theta_ref (the globally smoothed one) is used for omega and kappa calculations here.

dx = diff(x_ref); 
dy = diff(y_ref); 

if isempty(dx) || isempty(dy) 
    error('Path is too short for dx/dy calculation in curve detection (Step 6).');
end

dt = 0.05; % dt for this block's calculations (as per user's original block)
 
vx_calc_points = sqrt(dx.^2 + dy.^2) / dt;  
vx = [vx_calc_points(:); vx_calc_points(end)]; 

% omega calculation uses the globally defined and smoothed theta_ref from Step 5
omega = [0; diff(unwrap(theta_ref))]/dt; % theta_ref is already unwrapped if calculated from unwrap(theta_ref_calc)
if length(omega) > length(vx) 
    omega = omega(1:length(vx)); 
elseif length(vx) > length(omega)
     omega = [omega; repmat(omega(end), length(vx)-length(omega),1)]; 
end

kappa = abs(omega ./ (vx + 1e-3)); 
kappa = smoothdata(kappa, 'gaussian', 10); % Smoothing kappa as in user's block

% Parameters from user's "working before" block
kappa_threshold = 0.3; 
omega_spike_threshold = deg2rad(12); 
total_angle_thresh = deg2rad(10); 
min_curve_length = 30; 
default_padding = 20; 
sharp_padding = 50; 

% Parameters for spiky tail detection and extension
tail_percentage_for_spike_check = 0.30; 
omega_extend_threshold = deg2rad(10);    
samples_to_add_after_spike = 70; % Adjusted based on feedback     
min_length_for_tail_check = 25;       

is_curve = kappa > kappa_threshold; 
curve_bounds = []; 
in_curve = false; 

for i_loop = 1:length(is_curve) 
    if is_curve(i_loop) && ~in_curve 
        loop_start_idx = i_loop; 
        in_curve = true; 
    elseif (~is_curve(i_loop) || i_loop == length(is_curve)) && in_curve 
        loop_end_idx = i_loop - 1 + (i_loop == length(is_curve) && is_curve(i_loop)); 
        in_curve = false; 
        
        original_loop_end_idx = loop_end_idx; 

        % --- Spiky Tail Detection and Extension ---
        segment_len_raw_loop = original_loop_end_idx - loop_start_idx + 1;
        
        if segment_len_raw_loop >= min_length_for_tail_check 
            tail_check_start_offset = floor(segment_len_raw_loop * (1 - tail_percentage_for_spike_check));
            abs_tail_check_start_idx = loop_start_idx + max(0, tail_check_start_offset -1); 
            abs_tail_check_start_idx = min(abs_tail_check_start_idx, original_loop_end_idx); 

            if abs_tail_check_start_idx < original_loop_end_idx 
                omega_check_start = max(1, min(length(omega), abs_tail_check_start_idx));
                omega_check_end   = max(1, min(length(omega), original_loop_end_idx));
                
                if omega_check_start <= omega_check_end && (omega_check_end - omega_check_start +1) > 0
                    omega_in_tail_segment = omega(omega_check_start : omega_check_end);
                    if ~isempty(omega_in_tail_segment)
                        [max_abs_omega_in_tail, rel_idx_spike_in_tail] = max(abs(omega_in_tail_segment));
                        
                        if max_abs_omega_in_tail > omega_extend_threshold
                            abs_idx_spike = omega_check_start + rel_idx_spike_in_tail - 1;
                            
                            new_potential_end_idx = abs_idx_spike + samples_to_add_after_spike;
                            new_potential_end_idx = min(new_potential_end_idx, length(x_ref)); 
                            
                            if new_potential_end_idx > original_loop_end_idx
                                fprintf('    INFO: Spiky tail detected in raw curve %d-%d. Spike at %d (omega: %.2f rad/s). Original end: %d. Extending to new end_idx: %d\n', ...
                                        loop_start_idx, original_loop_end_idx, abs_idx_spike, omega(abs_idx_spike), original_loop_end_idx, new_potential_end_idx);
                                loop_end_idx = new_potential_end_idx; 
                            end
                        end
                    end
                end
            end
        end
        % --- End of Spiky Tail Extension ---

        safe_start_idx = max(1, min(loop_start_idx, length(theta_ref))); 
        safe_end_idx = max(1, min(loop_end_idx, length(theta_ref))); 
        
        safe_start_idx_omega = max(1, min(loop_start_idx, length(omega))); 
        safe_end_idx_omega = max(1, min(loop_end_idx, length(omega))); 

        if safe_start_idx <= safe_end_idx 
            current_segment_length_final = safe_end_idx - safe_start_idx + 1; 
            total_angle = abs(unwrap(theta_ref(safe_end_idx)) - unwrap(theta_ref(safe_start_idx))); 
            
            max_omega_val_final = 0; 
            if safe_start_idx_omega <= safe_end_idx_omega && ~isempty(omega(safe_start_idx_omega:safe_end_idx_omega))
                 max_omega_val_final = max(abs(omega(safe_start_idx_omega:safe_end_idx_omega))); 
            end 
            
            is_sharp_spike = max_omega_val_final > omega_spike_threshold; 
            is_sustained_turn = current_segment_length_final >= min_curve_length && ... 
                                total_angle > total_angle_thresh; 
            if is_sustained_turn || is_sharp_spike 
                curve_bounds = [curve_bounds; loop_start_idx, loop_end_idx, max_omega_val_final]; 
            end 
        end 
    end 
end 

% --- Merging and Padding Logic (with LESS AGGRESSIVE merging thresholds) ---
merged_bounds = []; 
if ~isempty(curve_bounds) 
    current_bound_candidate = curve_bounds(1,:); 
    for i = 2:size(curve_bounds,1) 
        next_bound_candidate = curve_bounds(i,:); 
        
        idx_curr_end = min(length(x_ref), current_bound_candidate(2));
        idx_next_start = min(length(x_ref), next_bound_candidate(1));
        idx_next_start = max(1, idx_next_start);

        end_point = [x_ref(idx_curr_end), y_ref(idx_curr_end)]; 
        start_point = [x_ref(idx_next_start), y_ref(idx_next_start)]; 
        spatial_gap = norm(end_point - start_point); 
        index_gap_val = next_bound_candidate(1) - current_bound_candidate(2); 

        % MODIFIED (LESS AGGRESSIVE) MERGING THRESHOLDS
        if (index_gap_val <= 40) || (spatial_gap < 1.25)  % Reduced from 75 & 2.0
            current_bound_candidate(2) = next_bound_candidate(2); 
            current_bound_candidate(3) = max(current_bound_candidate(3), next_bound_candidate(3)); 
        else 
            merged_bounds = [merged_bounds; current_bound_candidate]; 
            current_bound_candidate = next_bound_candidate; 
        end 
    end 
    merged_bounds = [merged_bounds; current_bound_candidate]; 
end 

final_bounds = [];
if ~isempty(merged_bounds) 
    idx_merge_loop = 1; 
    while idx_merge_loop <= size(merged_bounds,1) 
        current_final_bound = merged_bounds(idx_merge_loop,:); 
        idx_inner_merge_loop = idx_merge_loop + 1; 
        while idx_inner_merge_loop <= size(merged_bounds,1) 
            next_final_bound_start = merged_bounds(idx_inner_merge_loop,1); 
            
            idx_curr_final_end = min(length(x_ref), current_final_bound(2));
            idx_next_final_start = min(length(x_ref), next_final_bound_start);
            idx_next_final_start = max(1, idx_next_final_start);

            index_gap_final = next_final_bound_start - current_final_bound(2); 
            spatial_gap_final = norm([x_ref(idx_curr_final_end) y_ref(idx_curr_final_end)] - ... 
                                     [x_ref(idx_next_final_start) y_ref(idx_next_final_start)]); 
            
            % MODIFIED (LESS AGGRESSIVE) MERGING THRESHOLDS
            if index_gap_final <= 40 || spatial_gap_final < 1.25 % Reduced from 75 & 2.0
                current_final_bound(2) = merged_bounds(idx_inner_merge_loop,2); 
                current_final_bound(3) = max(current_final_bound(3), merged_bounds(idx_inner_merge_loop,3)); 
                idx_inner_merge_loop = idx_inner_merge_loop + 1; 
            else 
                break; 
            end 
        end 
        final_bounds = [final_bounds; current_final_bound]; 
        idx_merge_loop = idx_inner_merge_loop; 
    end 
end

curve_bounds_padded = []; 
for i_pad_loop = 1:size(final_bounds,1) 
    s = final_bounds(i_pad_loop,1); 
    e = final_bounds(i_pad_loop,2); 
    max_w = final_bounds(i_pad_loop,3); 
    remaining_samples = length(x_ref) - e; 
    pad = default_padding; 
    if max_w > omega_spike_threshold * 1.5 
        pad = sharp_padding; 
        if remaining_samples < 100 
            pad = min(pad, max(0, remaining_samples -5)); 
        end 
    end 
    start_idx_padded = max(1, s - pad); 
    if i_pad_loop > 1 && ~isempty(curve_bounds_padded) 
        prev_end = curve_bounds_padded(end,2); 
        start_idx_padded = max(start_idx_padded, prev_end + 1);  
    end 
    end_idx_padded = min(length(x_ref), e + pad); 
    if i_pad_loop < size(final_bounds,1) 
        next_start_for_padding_final = final_bounds(i_pad_loop+1,1); 
        end_idx_padded = min(end_idx_padded, next_start_for_padding_final - 1); 
    end 
    if start_idx_padded < end_idx_padded 
        curve_bounds_padded = [curve_bounds_padded; start_idx_padded, end_idx_padded]; 
    end 
end 
fprintf('  Detected %d curves for potential modification.\n', size(curve_bounds_padded,1));
%%% END MODIFICATION V2.3.9

% --- RESTORED Signal Plots ---
fprintf('Step 6.1: Plotting path signals (theta, omega, vx, kappa)...\n');
figure('Name', 'Path Signal Analysis');
% Plot signals calculated within Step 6 for consistency with curve detection
subplot(4,1,1); plot(theta_ref); title('\theta_{ref} (rad)'); ylabel('Angle (rad)'); grid on; 
subplot(4,1,2); plot(omega); title('\omega (rad/s)'); ylabel('Ang. Vel (rad/s)'); grid on; 
subplot(4,1,3); plot(vx); title('v_x (m/s)'); ylabel('Speed (m/s)'); grid on;    
subplot(4,1,4); plot(kappa); title('Curvature \kappa (1/m)'); ylabel('Curvature (1/m)'); xlabel('Sample Index'); grid on; 
linkaxes(findobj(gcf, 'Type', 'Axes'), 'x'); 
fprintf('  Signal plots generated.\n');
hold off;
% --- Debug Plot: Detected Curves ---
figure;
plot(x_ref, y_ref, 'k--', 'DisplayName', 'Reference Path'); hold on;
axis equal; title('Detected Curves for Modification');
if ~isempty(curve_bounds_padded)
    for i = 1:size(curve_bounds_padded,1)
        s_idx = curve_bounds_padded(i,1);
        e_idx = curve_bounds_padded(i,2);
        idx_range = s_idx:e_idx; 
        plot(x_ref(idx_range), y_ref(idx_range), 'm-', 'LineWidth', 2);
        mid_idx = round(mean(idx_range));
        text(x_ref(mid_idx), y_ref(mid_idx), sprintf('%d', i), ...
             'Color', 'blue', 'FontSize', 10, 'FontWeight', 'bold');
        plot(x_ref(s_idx), y_ref(s_idx), 'go', 'MarkerSize', 6, 'LineWidth', 1.5); 
        text(x_ref(s_idx), y_ref(s_idx), sprintf('S%d', s_idx), ...
             'Color', 'green', 'FontSize', 8, 'FontWeight', 'bold', 'VerticalAlignment', 'bottom');
        plot(x_ref(e_idx), y_ref(e_idx), 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);
        text(x_ref(e_idx), y_ref(e_idx), sprintf('E%d', e_idx), ...
             'Color', 'red', 'FontSize', 8, 'FontWeight', 'bold', 'VerticalAlignment', 'top');
    end
end
legend('Reference Path', 'Detected Curves (if any)');
hold off;

% --- Initial Simulation (Original Smoothed Path) ---
fprintf('Step 7: Simulating original smoothed path...\n');
[truck, trailer, target_xs, target_ys] = simulate_vehicle(x_ref, y_ref); 
fprintf('  Simulation complete.\n');
figure;
animate_vehicles_with_map(openField, x_ref, y_ref, truck, trailer, target_xs, target_ys);

% --- Path Modification ---
fprintf('Step 8: Modifying curves with Recker-Bézier approach...\n');
d1_param = 1.5; 
d2_param = 0.75; 

% Pass the unified theta_ref (calculated after smoothing in Step 5)
[x_ref_mod, y_ref_mod, overall_modification_successful] = modify_curves_bezier(x_ref, y_ref, theta_ref, curve_bounds_padded, openField, truck, trailer, d1_param, d2_param);

if ~overall_modification_successful
    fprintf('\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n');
    fprintf('!!! CRITICAL: Path modification failed to produce a feasible path for all curves.\n');
    fprintf('!!! Path is UNFEASIBLE. Consider re-planning or adjusting parameters.\n');
    fprintf('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n');
else
    fprintf('\nPath modification completed. All segments reported as successfully processed (or did not require modification).\n');
end

% --- Simulation of Modified Path ---
fprintf('Step 9: Simulating modified path...\n');
[truck_mod, trailer_mod, target_xs_mod, target_ys_mod] = simulate_vehicle(x_ref_mod, y_ref_mod);
fprintf('  Simulation of modified path complete.\n');

% --- Final Visualization ---
fprintf('Step 10: Visualizing final results...\n');
figure('Name','Final Simulation Comparison');
show(openField); hold on;
plot(x_ref, y_ref, 'k--', 'DisplayName', 'Original Smoothed Path');
plot(x_ref_mod, y_ref_mod, 'b-', 'LineWidth', 2, 'DisplayName', 'Modified Path');
plot(truck_mod.x, truck_mod.y, 'c-', 'DisplayName', 'Truck on Modified Path');
plot(trailer_mod.x, trailer_mod.y, 'r-', 'DisplayName', 'Trailer on Modified Path');
legend('Location', 'best');
axis equal;
title('Path Comparison: Original, Modified, and Simulated Vehicle');
hold off;

animate_vehicles_with_map(openField, x_ref_mod, y_ref_mod, truck_mod, trailer_mod, target_xs_mod, target_ys_mod);
fprintf('--- Script Finished ---\n');

% Placeholder for geom() if it's defined elsewhere, or ensure it's in matlab_bezier_refactored
function g = geom() % Added placeholder geom if not available globally
    g.Ltr  = 2.20;   
    g.Wtr  = 1.00;   
    g.Ltl  = 1.50;   
    g.Wtl  = 1.00;   
    g.Lh   = 2.00;   
    g.dt   = 0.05;   
    g.v    = 0.50;
    g.look = 1.00;   
end
