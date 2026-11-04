%% modify_curves_bezier.m
% This function modifies a reference path curve-by-curve using Bézier curves
% to resolve vehicle collisions or near-misses identified in a simulation.
% It has been refactored to use standardized safety checks and includes
% integrated utility functions for simulation and evaluation.
%
%SYNTAX:
% [x_ref_mod, y_ref_mod, overall_modification_successful] = ...
%     modify_curves_bezier(x_ref, y_ref, theta_ref, ...
%                          curve_bounds_padded, ...
%                          map, ...
%                          truck_orig_sim, trailer_orig_sim, ...
%                          g_geom, RO_truck_param, RO_trailer_param, ...
%                          app_costmaps, ...
%                          truck_dims_obj, trailer_dims_obj, ...
%                          global_hotspots, ...
%                          danger_original_segment_optional_main_call)
%
function [x_ref_mod, y_ref_mod, overall_modification_successful] = ...
    modify_curves_bezier(x_ref, y_ref, theta_ref, ...
                         curve_bounds_padded, ...
                         map, ...                     % map is binaryOccupancyMap for plotting
                         truck_orig_sim, trailer_orig_sim, ...
                         g_geom, RO_truck_param, RO_trailer_param, ...
                         app_costmaps, ...
                         truck_dims_obj, trailer_dims_obj, ...
                         global_hotspots, ...
                         danger_original_segment_optional_main_call)
    
    % --- Argument Handling & Initial Setup ---
    
    % The 15th argument is optional
    if nargin < 16 || isempty(danger_original_segment_optional_main_call)
        danger_original_segment_optional_main_call = ...
            struct('truck_collision', 0, 'truck_close', 0, 'trailer_collision', 0, 'trailer_close', 0);
    end
    
    % Ensure reference path vectors are column vectors.
    x_ref = x_ref(:);
    y_ref = y_ref(:);
    theta_ref = unwrap(theta_ref(:));
    
    % Initialize output variables.
    x_ref_mod = [];
    y_ref_mod = [];
    last_idx = 1; % Keeps track of the last stitched index.
    overall_modification_successful = true;

    % --- Visualization Setup ---
    
    figure_handle = figure('Name', 'Recker Curve Evaluation (Refactored Safety Checks)');
    show(map);
    hold on;
    axis equal;
    grid on;
    title('Path Modification Diagnostics (Refactored Safety Checks)');

    % Plot the original, unmodified path.
    plot(x_ref, y_ref, 'k--', 'DisplayName', 'Original Path');
    
    % Initialize placeholder plots for initial collision points.
    h_initial_truck_collision_plot = plot(NaN, NaN, 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Initial Truck Collision');
    h_initial_trailer_collision_plot = plot(NaN, NaN, 'mx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Initial Trailer Collision');

    % Set up the legend with initial handles.
    legend_handles = [findobj(gca, 'DisplayName', 'Original Path'), ...
                      h_initial_truck_collision_plot, h_initial_trailer_collision_plot];
    legend_handles = legend_handles(isgraphics(legend_handles)); % Ensure only valid handles are stored.

    % --- Simulation & Kinematic Parameter Calculation ---

    % Calculate the number of samples the trailer lags behind the truck.
    trailer_lag_distance = g_geom.Lh + g_geom.Ltl;
    distance_per_sample_step = g_geom.v * g_geom.dt;
    
    if distance_per_sample_step <= 1e-6
        num_lag_samples = 0;
        warning('geom:lag_calc', 'Distance per sample step is very small or zero; num_lag_samples set to 0.');
    else
        num_lag_samples = ceil(trailer_lag_distance / distance_per_sample_step);
    end
    fprintf('Trailer lag samples: %d (based on %.2fm lag distance and %.3fm/sample)\n', num_lag_samples, trailer_lag_distance, distance_per_sample_step);
    
    current_curve_final_diag_handles = [];

    % --- Main Loop: Process Each Curve Segment ---
    
    for c = 1:size(curve_bounds_padded, 1)
        
        % Clear diagnostic graphics from the previous curve iteration.
        if ~isempty(current_curve_final_diag_handles) && all(isgraphics(current_curve_final_diag_handles))
            delete(current_curve_final_diag_handles);
        end
        current_curve_final_diag_handles = [];
        
        fprintf('\n--- CURVE %d of %d ---\n', c, size(curve_bounds_padded, 1));
        
        % --- Segment Definition & Validation ---
        
        s_idx_nominal_curve = curve_bounds_padded(c, 1);
        e_idx_nominal_curve = curve_bounds_padded(c, 2);
        
        % Skip if the curve indices are invalid.
        if s_idx_nominal_curve >= e_idx_nominal_curve || s_idx_nominal_curve < 1 || e_idx_nominal_curve > length(x_ref)
            fprintf(' Skipping invalid nominal curve segment %d-%d.\n', s_idx_nominal_curve, e_idx_nominal_curve);
            
            % Stitch the original path up to this point to avoid gaps.
            safe_e_idx_stitch = min(e_idx_nominal_curve, length(x_ref));
            if last_idx <= safe_e_idx_stitch && last_idx <= length(x_ref)
                x_ref_mod = [x_ref_mod; x_ref(last_idx:safe_e_idx_stitch)];
                y_ref_mod = [y_ref_mod; y_ref(last_idx:safe_e_idx_stitch)];
                last_idx = safe_e_idx_stitch + 1;
            elseif last_idx > safe_e_idx_stitch && safe_e_idx_stitch >= curve_bounds_padded(c, 1)
                 last_idx = safe_e_idx_stitch + 1;
            end
            continue;
        end
        
        x_curve_nominal_plot = x_ref(s_idx_nominal_curve:e_idx_nominal_curve);
        y_curve_nominal_plot = y_ref(s_idx_nominal_curve:e_idx_nominal_curve);
        
        % Define the input segment for the Recker modification algorithm.
        s_idx_recker_input = s_idx_nominal_curve;
        e_idx_recker_input = e_idx_nominal_curve;
        
        % Extend very short curves to provide more context for the algorithm.
        current_arc_len_nominal = sum(hypot(diff(x_curve_nominal_plot), diff(y_curve_nominal_plot)));
        if current_arc_len_nominal < 1.2 * (g_geom.Ltr + g_geom.Ltl) && s_idx_recker_input > 5 && e_idx_recker_input < length(x_ref) - 5
            s_idx_recker_input_orig_log = s_idx_recker_input;
            e_idx_recker_input_orig_log = e_idx_recker_input;
            s_idx_recker_input = s_idx_recker_input - 5;
            e_idx_recker_input = e_idx_recker_input + 5;
            fprintf(' Nominal curve: %d-%d. Tiny-corner extension for Recker input: %d-%d -> %d-%d.\n', ...
                s_idx_nominal_curve, e_idx_nominal_curve, s_idx_recker_input_orig_log, e_idx_recker_input_orig_log, s_idx_recker_input, e_idx_recker_input);
        else
            fprintf(' Nominal curve: %d-%d. Recker input segment: %d-%d.\n', ...
                s_idx_nominal_curve, e_idx_nominal_curve, s_idx_recker_input, e_idx_recker_input);
        end
        
        x_segment_for_recker_mod = x_ref(s_idx_recker_input:e_idx_recker_input);
        y_segment_for_recker_mod = y_ref(s_idx_recker_input:e_idx_recker_input);
        
        % Plot the nominal curve segment being evaluated.
        h_nom_curve_plot = plot(x_curve_nominal_plot, y_curve_nominal_plot, 'm-', 'LineWidth', 1.5, 'DisplayName', sprintf('C%d Nom.', c));
        text(x_curve_nominal_plot(round(end/2)), y_curve_nominal_plot(round(end/2)), ...
             sprintf('%d', c), 'Color', 'm', 'FontWeight', 'bold', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
        
        % Safely add the nominal curve plot handle to the legend.
        valid_temp_legend_handles = legend_handles(isgraphics(legend_handles));
        add_to_legend = true;
        if ~isempty(valid_temp_legend_handles)
            existing_display_names = get(valid_temp_legend_handles, 'DisplayName');
            if ~iscell(existing_display_names) && ~isempty(existing_display_names)
                existing_display_names = {existing_display_names};
            elseif isempty(existing_display_names)
                 existing_display_names = {};
            end
            if any(strcmp(existing_display_names, get(h_nom_curve_plot, 'DisplayName')))
                add_to_legend = false;
            end
        end
        if add_to_legend
            legend_handles(end+1) = h_nom_curve_plot;
        end
        
        % --- Initial Safety Evaluation of the Original Segment ---
        
        % Find hard collisions within this curve segment from the global hotspot list.
        curve_specific_hard_collisions_from_global = [];
        if ~isempty(global_hotspots)
            for k_hot = 1:length(global_hotspots)
                if global_hotspots(k_hot).global_path_idx >= s_idx_recker_input && ...
                   global_hotspots(k_hot).global_path_idx <= e_idx_recker_input && ...
                   global_hotspots(k_hot).is_hard_collision
                    curve_specific_hard_collisions_from_global = [curve_specific_hard_collisions_from_global, global_hotspots(k_hot)];
                end
            end
        end
        
        % Define the local window for safety evaluation, including trailer lag.
        eval_s_idx_local = s_idx_nominal_curve;
        temp_eval_e_idx_local = e_idx_nominal_curve + num_lag_samples;
        eval_e_idx_local = min(temp_eval_e_idx_local, length(x_ref));
        
        % Ensure the evaluation window does not extend into the next curve.
        if c < size(curve_bounds_padded, 1)
            s_idx_next_curve_local = curve_bounds_padded(c + 1, 1);
            if s_idx_next_curve_local > eval_s_idx_local && s_idx_next_curve_local <= length(x_ref)
                eval_e_idx_local = min(eval_e_idx_local, s_idx_next_curve_local - 1);
            end
        end
        eval_e_idx_local = max(eval_e_idx_local, eval_s_idx_local);
        
        fprintf(' Local Safety Eval Window (orig. path for C%d): %d-%d.\n', c, eval_s_idx_local, eval_e_idx_local);
        
        status_segment_local_eval = [];
        danger_metrics_segment_local_eval = struct('truck_collision', 0, 'truck_close', 0, 'trailer_collision', 0, 'trailer_close', 0);
        initial_collision_points_truck_x_plot = [];
        initial_collision_points_truck_y_plot = [];
        initial_collision_points_trailer_x_plot = [];
        initial_collision_points_trailer_y_plot = [];
        
        % Perform the detailed safety evaluation if the window is valid.
        if eval_s_idx_local <= eval_e_idx_local && ... 
           eval_s_idx_local > 0 && eval_e_idx_local > 0 && ...
           eval_s_idx_local <= length(truck_orig_sim.x) && eval_e_idx_local <= length(truck_orig_sim.x) && ...
           eval_s_idx_local <= length(trailer_orig_sim.x) && eval_e_idx_local <= length(trailer_orig_sim.x)
           
            current_segment_truck_geom.x = truck_orig_sim.x(eval_s_idx_local:eval_e_idx_local);
            current_segment_truck_geom.y = truck_orig_sim.y(eval_s_idx_local:eval_e_idx_local);
            current_segment_truck_geom.theta = truck_orig_sim.theta(eval_s_idx_local:eval_e_idx_local);
            current_segment_trailer_geom.x = trailer_orig_sim.x(eval_s_idx_local:eval_e_idx_local);
            current_segment_trailer_geom.y = trailer_orig_sim.y(eval_s_idx_local:eval_e_idx_local);
            current_segment_trailer_geom.theta = trailer_orig_sim.theta(eval_s_idx_local:eval_e_idx_local);
            
            fprintf(' Evaluating original path for C%d (local window %d-%d) using standardized safety check.\n', c, eval_s_idx_local, eval_e_idx_local);
            [status_segment_local_eval, danger_metrics_segment_local_eval] = ...
                evaluate_segment_safety_detailed(current_segment_truck_geom, current_segment_trailer_geom, ...
                                                 g_geom, RO_truck_param, RO_trailer_param, app_costmaps);
            
            % =========================================================================
            % --- DEBUG SECTION (Investigate first reported truck collision) ---
            % This section provides detailed diagnostics if a truck collision is
            % detected in the original segment for the first curve.
            if c == 1 && ~isempty(status_segment_local_eval) && danger_metrics_segment_local_eval.truck_collision > 0
                fprintf('DEBUG_LOG: Investigating Curve 1 - First reported truck collision in original segment.\n');
                first_dbg_truck_collision_idx_in_segment = -1;
                for dbg_i = 1:length(status_segment_local_eval)
                    if status_segment_local_eval(dbg_i).truck_flag == "collision"
                        first_dbg_truck_collision_idx_in_segment = dbg_i;
                        break;
                    end
                end
                
                if first_dbg_truck_collision_idx_in_segment > 0
                    actual_sim_index = eval_s_idx_local + first_dbg_truck_collision_idx_in_segment - 1;
                    fprintf('DEBUG_LOG: Examining original sim index: %d (from segment eval index: %d)\n', actual_sim_index, first_dbg_truck_collision_idx_in_segment);
                    
                    truck_geom_x_dbg = truck_orig_sim.x(actual_sim_index);
                    truck_geom_y_dbg = truck_orig_sim.y(actual_sim_index);
                    truck_geom_theta_rad_dbg = truck_orig_sim.theta(actual_sim_index);
                    
                    [truck_ra_x_dbg, truck_ra_y_dbg, truck_ra_theta_deg_dbg] = convertToRearAxlePose(...
                        truck_geom_x_dbg, truck_geom_y_dbg, truck_geom_theta_rad_dbg, ...
                        g_geom.Ltr, RO_truck_param);
                    truck_pose_ra_deg_dbg = [truck_ra_x_dbg, truck_ra_y_dbg, truck_ra_theta_deg_dbg];
                    
                    isTruckHard_dbg_check = checkOccupied(app_costmaps.truck_hard, truck_pose_ra_deg_dbg);
                    fprintf('DEBUG_LOG: checkOccupied(truck_hard_costmap, pose [%.3f, %.3f, %.2f deg]) result: %d\n', ...
                            truck_pose_ra_deg_dbg(1), truck_pose_ra_deg_dbg(2), truck_pose_ra_deg_dbg(3), isTruckHard_dbg_check);
                            
                    % Visualize the specific pose and costmap for debugging.
                    dbg_fig_handle = figure('Name', sprintf('DEBUG: Pose Check SimIdx %d (VehicleCostmap Overlay)', actual_sim_index));
                    ax_dbg = axes(dbg_fig_handle);
                    
                    plot(app_costmaps.truck_hard, 'Parent', ax_dbg, 'Inflation', 'on'); % Show inflated areas
                    hold(ax_dbg, 'on');
                    axis(ax_dbg, 'equal');
                    
                    % Zoom in on the vehicle area.
                    buffer = max(truck_dims_obj.Length, truck_dims_obj.Width) * 1.0; 
                    zoom_xlim = [truck_pose_ra_deg_dbg(1) - buffer, truck_pose_ra_deg_dbg(1) + buffer];
                    zoom_ylim = [truck_pose_ra_deg_dbg(2) - buffer, truck_pose_ra_deg_dbg(2) + buffer];
                    xlim(ax_dbg, zoom_xlim);
                    ylim(ax_dbg, zoom_ylim);
                    title(ax_dbg, sprintf('Truck (Idx %d, Collision: %d) on VehicleCostmap (Cell=%.2fm)', actual_sim_index, isTruckHard_dbg_check, app_costmaps.truck_hard.CellSize));
                    
                    % Plot footprints and reference points.
                    plotVehicleFootprint(truck_dims_obj, truck_pose_ra_deg_dbg, 'b', 'Truck Footprint (Reported Collision)', ax_dbg);
                    plot(ax_dbg, truck_geom_x_dbg, truck_geom_y_dbg, 'co', 'MarkerFaceColor', 'c', 'MarkerSize', 8, 'DisplayName', 'Truck Geom Center');
                    plot(ax_dbg, truck_ra_x_dbg, truck_ra_y_dbg, 'yo', 'MarkerFaceColor', 'y', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Truck Rear Axle (Ref/Center Point)');
                    
                    legend(ax_dbg, 'show', 'Location', 'eastoutside');
                    hold(ax_dbg, 'off');
                    
                    fprintf('DEBUG_LOG: Inspect plot. Does YELLOW truck rear axle dot fall on a LIGHT RED (inflated) region?\n');
                    fprintf('DEBUG_LOG: Inspect zoomed plot. Does BLUE truck footprint overlap BLACK obstacle cells?\n');
                    
                    keyboard; % Pause execution for manual inspection.
                else
                    fprintf('DEBUG_LOG: Curve 1 - No truck collisions were flagged by evaluate_segment_safety_detailed for its original segment.\n');
                end
            end
            % --- END DEBUG SECTION ---
            % =========================================================================

            % Collect collision points for visualization.
            for ii_local_plot = 1:length(status_segment_local_eval)
                k_global_plot = eval_s_idx_local + ii_local_plot - 1;
                if k_global_plot > 0 && k_global_plot <= length(truck_orig_sim.x)
                    if status_segment_local_eval(ii_local_plot).truck_flag == "collision"
                        initial_collision_points_truck_x_plot(end+1) = truck_orig_sim.x(k_global_plot);
                        initial_collision_points_truck_y_plot(end+1) = truck_orig_sim.y(k_global_plot);
                    end
                    if status_segment_local_eval(ii_local_plot).trailer_flag == "collision"
                        initial_collision_points_trailer_x_plot(end+1) = trailer_orig_sim.x(k_global_plot);
                        initial_collision_points_trailer_y_plot(end+1) = trailer_orig_sim.y(k_global_plot);
                    end
                end
            end
        else
            fprintf(' Warning: Local safety eval window for C%d (%d-%d) is too small or invalid. Assuming locally safe.\n', c, eval_s_idx_local, eval_e_idx_local);
        end
        
        % --- Determine if Modification is Needed ---
        
        needs_modification_due_to_global_hotspot = ~isempty(curve_specific_hard_collisions_from_global);
        
        if ~isempty(status_segment_local_eval)
            local_eval_truck_flags_str = string({status_segment_local_eval.truck_flag});
            local_eval_trailer_flags_str = string({status_segment_local_eval.trailer_flag});
            local_eval_turning_flags_logical = [status_segment_local_eval.is_turning];
        else
            local_eval_truck_flags_str = string([]);
            local_eval_trailer_flags_str = string([]);
            local_eval_turning_flags_logical = logical([]);
        end
        
        % Trigger modification if there is a "close" call during a turn.
        needs_modification_due_to_local_close_call_in_turn = false;
        if ~isempty(local_eval_turning_flags_logical)
            needs_modification_due_to_local_close_call_in_turn = ...
                any(local_eval_turning_flags_logical & ...
                    (strcmp(local_eval_truck_flags_str, "close") | strcmp(local_eval_trailer_flags_str, "close")));
        end
        
        needs_modification = needs_modification_due_to_global_hotspot || needs_modification_due_to_local_close_call_in_turn;
        danger_metrics_for_recker = danger_metrics_segment_local_eval;
        
        % Aggregate danger metrics from both global and local evaluations.
        if needs_modification_due_to_global_hotspot
            fprintf(' INFO: Curve %d (Recker input %d-%d) has %d hard collisions from global hotspot list.\n', c, s_idx_recker_input, e_idx_recker_input, length(curve_specific_hard_collisions_from_global));
            for k_h = 1:length(curve_specific_hard_collisions_from_global)
                if strcmp(curve_specific_hard_collisions_from_global(k_h).vehicle_type, 'truck')
                    danger_metrics_for_recker.truck_collision = danger_metrics_for_recker.truck_collision + 1;
                    initial_collision_points_truck_x_plot(end+1) = curve_specific_hard_collisions_from_global(k_h).vehicle_pose(1);
                    initial_collision_points_truck_y_plot(end+1) = curve_specific_hard_collisions_from_global(k_h).vehicle_pose(2);
                elseif strcmp(curve_specific_hard_collisions_from_global(k_h).vehicle_type, 'trailer')
                    danger_metrics_for_recker.trailer_collision = danger_metrics_for_recker.trailer_collision + 1;
                    initial_collision_points_trailer_x_plot(end+1) = curve_specific_hard_collisions_from_global(k_h).vehicle_pose(1);
                    initial_collision_points_trailer_y_plot(end+1) = curve_specific_hard_collisions_from_global(k_h).vehicle_pose(2);
                end
            end
        end
        
        if needs_modification_due_to_local_close_call_in_turn && ~needs_modification_due_to_global_hotspot
             fprintf(' INFO: Curve %d (Recker input %d-%d) triggered modification due to local close call during turn (no global hard collision in this segment).\n', c, s_idx_recker_input, e_idx_recker_input);
        end
        
        fprintf(' Initial Status for C%d (Recker input %d-%d): Needs Mod = %s. Recker Danger Metrics: (TC:%d, TrC:%d, TkC:%d, TlC:%d)\n', ...
            c, s_idx_recker_input, e_idx_recker_input, ...
            matlab.lang.makeValidName(string(needs_modification)), ...
            danger_metrics_for_recker.truck_collision, ...
            danger_metrics_for_recker.trailer_collision, ...
            danger_metrics_for_recker.truck_close, ...
            danger_metrics_for_recker.trailer_close);
        
        % Update collision plots with initial collision points.
        set(h_initial_truck_collision_plot, 'XData', unique(initial_collision_points_truck_x_plot, 'stable'), 'YData', unique(initial_collision_points_truck_y_plot, 'stable'));
        set(h_initial_trailer_collision_plot, 'XData', unique(initial_collision_points_trailer_x_plot, 'stable'), 'YData', unique(initial_collision_points_trailer_y_plot, 'stable'));
        
        % --- Apply Recker Modification or Keep Original Segment ---
        
        current_segment_modification_successful = true;
        if needs_modification
            fprintf(' Status: Unsafe. Applying Recker to segment %d-%d.\n', s_idx_recker_input, e_idx_recker_input);
            [x_mod, y_mod, diag_handles_from_recker, current_segment_modification_successful] = ...
                apply_recker_curve(x_segment_for_recker_mod, y_segment_for_recker_mod, ...
                                   theta_ref(s_idx_recker_input), theta_ref(e_idx_recker_input), map, ...
                                   truck_orig_sim, trailer_orig_sim, s_idx_recker_input, ...
                                   danger_metrics_for_recker, ...
                                   g_geom, RO_truck_param, RO_trailer_param, app_costmaps); 
            
            if ~current_segment_modification_successful
                fprintf(' ! WARNING: Recker modification for curve %d (segment %d-%d) FAILED to find a collision-free path.\n', ...
                        c, s_idx_recker_input, e_idx_recker_input);
                overall_modification_successful = false;
            else
                fprintf(' \u2714 Recker modification for curve %d (segment %d-%d) SUCCEEDED in finding a collision-free path.\n', c, s_idx_recker_input, e_idx_recker_input);
                current_curve_final_diag_handles = diag_handles_from_recker;
            end
            
            % Plot the modified curve segment.
            h_mod_curve_plot = plot(x_mod, y_mod, 'g-', 'LineWidth', 2, 'DisplayName', sprintf('C%d Mod.', c));
            
            % Safely add the modified curve plot handle to the legend.
            valid_temp_legend_handles = legend_handles(isgraphics(legend_handles));
            add_to_legend = true;
            if ~isempty(valid_temp_legend_handles)
                existing_display_names = get(valid_temp_legend_handles, 'DisplayName');
                 if ~iscell(existing_display_names) && ~isempty(existing_display_names)
                    existing_display_names = {existing_display_names};
                elseif isempty(existing_display_names)
                    existing_display_names = {};
                end
                if any(strcmp(existing_display_names, get(h_mod_curve_plot, 'DisplayName')))
                    add_to_legend = false;
                end
            end
            if add_to_legend
                legend_handles(end+1) = h_mod_curve_plot;
            end
        else
            fprintf(' \u2713 Status: Safe. Using segment %d-%d as is.\n', s_idx_recker_input, e_idx_recker_input);
            x_mod = x_segment_for_recker_mod;
            y_mod = y_segment_for_recker_mod;
        end
        
        fprintf(' Output segment (global %d-%d) has %d points.\n', s_idx_recker_input, e_idx_recker_input, length(x_mod));
        
        % --- Stitch Path Segments Together ---
        
        % Stitch the gap between the last segment and the current one.
        if last_idx < s_idx_recker_input
            gap_s_stitch = last_idx;
            gap_e_stitch = s_idx_recker_input - 1;
            if gap_s_stitch <= gap_e_stitch && gap_s_stitch > 0 && gap_e_stitch <= length(x_ref)
                x_ref_mod = [x_ref_mod; x_ref(gap_s_stitch:gap_e_stitch)];
                y_ref_mod = [y_ref_mod; y_ref(gap_s_stitch:gap_e_stitch)];
            end
        end
        
        % Append the current (potentially modified) segment.
        if ~isempty(x_mod)
            x_ref_mod = [x_ref_mod; x_mod(:)];
            y_ref_mod = [y_ref_mod; y_mod(:)];
        else
            warning('geom:empty_mod', 'x_mod for curve %d was empty. Stitching original Recker input segment.', c);
            x_ref_mod = [x_ref_mod; x_segment_for_recker_mod(:)];
            y_ref_mod = [y_ref_mod; y_segment_for_recker_mod(:)];
            overall_modification_successful = false;
        end
        
        last_idx = e_idx_recker_input + 1;
        drawnow; % Update plot in real-time.
        
    end % End of main loop for each curve.
    
    % --- Finalize Path and Plotting ---
    
    % Append any remaining portion of the original path.
    if last_idx <= length(x_ref)
        x_ref_mod = [x_ref_mod; x_ref(last_idx:end)];
        y_ref_mod = [y_ref_mod; y_ref(last_idx:end)];
    end
    
    % Plot the final, fully stitched path.
    h_final_path_plot = plot(x_ref_mod, y_ref_mod, 'b-', 'LineWidth', 2, 'DisplayName', 'Final Path');
    
    % Safely add the final path plot handle to the legend.
    valid_temp_legend_handles = legend_handles(isgraphics(legend_handles));
    add_to_legend = true;
    if ~isempty(valid_temp_legend_handles)
        existing_display_names = get(valid_temp_legend_handles, 'DisplayName');
        if ~iscell(existing_display_names) && ~isempty(existing_display_names)
            existing_display_names = {existing_display_names};
        elseif isempty(existing_display_names)
            existing_display_names = {};
        end
        if any(strcmp(existing_display_names, get(h_final_path_plot, 'DisplayName')))
            add_to_legend = false;
        end
    end
    if add_to_legend
        legend_handles(end+1) = h_final_path_plot;
    end
    
    % Display the legend, ensuring no duplicate entries.
    try
        valid_legend_handles_for_legend_cmd = legend_handles(isgraphics(legend_handles));
        if ~isempty(valid_legend_handles_for_legend_cmd)
            display_names_for_legend_cmd = get(valid_legend_handles_for_legend_cmd, 'DisplayName');
            if ~iscell(display_names_for_legend_cmd) && ~isempty(display_names_for_legend_cmd)
                 display_names_for_legend_cmd = {display_names_for_legend_cmd};
            elseif isempty(display_names_for_legend_cmd)
                display_names_for_legend_cmd = {};
            end
            [~, ia, ~] = unique(display_names_for_legend_cmd, 'stable');
            if ~isempty(ia)
                legend(valid_legend_handles_for_legend_cmd(ia), 'Location', 'bestoutside');
            end
        end
    catch ME_legend
        warning('Could not update legend: %s', ME_legend.message);
    end
    
    hold off;
    
    % --- Final Status Report ---
    fprintf('\n--- Path Modification Complete (Refactored Safety Checks) ---\n');
    if ~overall_modification_successful
        fprintf('--- OVERALL PATH MODIFICATION FAILED TO RESOLVE ALL ISSUES. ---\n');
    else
        fprintf('--- OVERALL PATH MODIFICATION SUCCEEDED. ---\n');
    end
    fprintf('DEBUG_LOG: Reached the end of modify_curves_bezier. overall_modification_successful is %d\n', overall_modification_successful);
    
end % End of main function modify_curves_bezier.


% =========================================================================
% =========================================================================
% --- NESTED AND LOCAL FUNCTIONS ---
% =========================================================================
% =========================================================================


%% --- Nested Function: apply_recker_curve ---
% Applies an iterative Bézier curve modification algorithm to find a
% collision-free path for a given segment.
function [x_best, y_best, final_diagnostic_handles, modification_succeeded] = ...
    apply_recker_curve(x_curve_segment_input, y_curve_segment_input, ...
                       thetaS_segment_input, thetaE_segment_input, ...
                       map_env, ... 
                       truck_original_full_sim, trailer_original_full_sim, ...
                       s_idx_global_start_of_recker_segment, ...
                       danger_metrics_for_original_recker_segment, ...
                       g_geom, RO_truck_param, RO_trailer_param, app_costmaps)

    % --- Algorithm Constants ---
    OUTER_WALL_HIT_PENALTY      = 7;
    LOOP_PENALTY                = 1000;
    COLLISION_PENALTY_ABSOLUTE  = 1e7;
    D1_ARC_LIMIT_FACTOR         = 0.5;
    D1_RECKER_BASE_FACTOR       = 0.5;
    D2_BASE_MIN_FACTOR_OF_D1    = 0.3;
    MAX_ATTEMPTS_PER_SIDE       = 16;
    LONG_CURVE_ARC_THRESHOLD    = 4.0;
    INITIAL_D2_SCALE_FACTOR_ONLY_CLOSE  = 0.20;
    INITIAL_D2_SCALE_FACTOR_COLLISION   = 0.8;
    D2_GROWTH_FACTOR_ONLY_CLOSE         = 1.05;
    D2_GROWTH_FACTOR_COLLISION          = 1.18;
    D2_GROWTH_ADJUSTMENT_FACTOR_LONG_CURVE = 0.95;
    MIN_KAPPA_FOR_D2_FORMULA           = 0.05;
    D2_BASE_FOR_VERY_LOW_KAPPA         = 0.10;

    % --- Initialize Bézier Parameters (d1, d2) based on segment geometry ---
    Ltot_recker = g_geom.Ltr + g_geom.Lh + g_geom.Ltl;
    arc_segment_recker = sum(hypot(diff(x_curve_segment_input), diff(y_curve_segment_input)));
    delta_angle_segment_recker = abs(wrapToPi(thetaE_segment_input - thetaS_segment_input));
    
    d1_recker_comp = D1_RECKER_BASE_FACTOR * Ltot_recker;
    d1_arc_comp = D1_ARC_LIMIT_FACTOR * arc_segment_recker;
    
    if arc_segment_recker < 1e-2 % Handle straight or near-zero length segments
        d1_handle_fixed_recker = min(d1_recker_comp, g_geom.Ltr);
        kappa_segment_recker = 2; % High kappa to avoid small d2
    else
        d1_handle_fixed_recker = min(d1_recker_comp, d1_arc_comp);
        kappa_segment_recker = delta_angle_segment_recker / arc_segment_recker;
    end
    d1_handle_fixed_recker = max(d1_handle_fixed_recker, 0.1 * Ltot_recker);
    
    Rmin_approx_recker = (g_geom.Lh + g_geom.Ltl) / tan(deg2rad(35));
    if kappa_segment_recker < MIN_KAPPA_FOR_D2_FORMULA
        d2_iterative_base_recker = D2_BASE_FOR_VERY_LOW_KAPPA;
    else
        d2_iterative_base_recker = max(0.15, 0.8 * (1 / kappa_segment_recker - Rmin_approx_recker));
        if kappa_segment_recker > 0.4 || arc_segment_recker < 1.5 * d1_handle_fixed_recker
            min_d2_boosted_recker = D2_BASE_MIN_FACTOR_OF_D1 * d1_handle_fixed_recker;
            if d2_iterative_base_recker < min_d2_boosted_recker
                d2_iterative_base_recker = min_d2_boosted_recker;
            end
        end
    end
    
    % Adjust initial d2 scaling and growth based on the type of initial danger.
    base_d2_growth_recker = D2_GROWTH_FACTOR_COLLISION;
    initial_d2_scale_recker = INITIAL_D2_SCALE_FACTOR_COLLISION;
    if ~isempty(danger_metrics_for_original_recker_segment)
        is_initial_col_recker = (danger_metrics_for_original_recker_segment.truck_collision > 0 || ...
                                 danger_metrics_for_original_recker_segment.trailer_collision > 0);
        sum_close_recker = danger_metrics_for_original_recker_segment.truck_close + ...
                           danger_metrics_for_original_recker_segment.trailer_close;
                           
        if ~is_initial_col_recker && sum_close_recker > 0
            initial_d2_scale_recker = INITIAL_D2_SCALE_FACTOR_ONLY_CLOSE;
            base_d2_growth_recker = D2_GROWTH_FACTOR_ONLY_CLOSE;
        elseif is_initial_col_recker
            initial_d2_scale_recker = INITIAL_D2_SCALE_FACTOR_COLLISION;
            base_d2_growth_recker = D2_GROWTH_FACTOR_COLLISION;
        end
    end
    
    d2_iterative_base_recker = d2_iterative_base_recker * initial_d2_scale_recker;
    d2_iterative_base_recker = max(d2_iterative_base_recker, 0.025);
    
    % --- Iteration Setup ---
    
    % Determine directions to try (e.g., left/right of the original path).
    dirs_to_try_recker = [sign(wrapToPi(thetaS_segment_input - thetaE_segment_input)), ...
                          -sign(wrapToPi(thetaS_segment_input - thetaE_segment_input))];
    if dirs_to_try_recker(1) == 0, dirs_to_try_recker(1) = 1; end
    if length(dirs_to_try_recker) > 1 && dirs_to_try_recker(2) == 0, dirs_to_try_recker(2) = -1; end
    if length(dirs_to_try_recker) > 1 && dirs_to_try_recker(1) == dirs_to_try_recker(2)
        dirs_to_try_recker = [1, -1];
    end
    
    % Calculate the score of the original, unmodified segment.
    stats_input_recker_segment = danger_metrics_for_original_recker_segment;
    score_input_recker_segment = 0;
    if stats_input_recker_segment.truck_collision > 0 || stats_input_recker_segment.trailer_collision > 0
        score_input_recker_segment = score_input_recker_segment + COLLISION_PENALTY_ABSOLUTE * ...
                                     (stats_input_recker_segment.truck_collision + stats_input_recker_segment.trailer_collision);
    end
    score_input_recker_segment = score_input_recker_segment + ...
                                 (stats_input_recker_segment.truck_close + stats_input_recker_segment.trailer_close);
    
    % Initialize the best solution with the original segment.
    bestOverallScore_recker = score_input_recker_segment;
    x_best = x_curve_segment_input;
    y_best = y_curve_segment_input;
    best_p1_overall_recker = [NaN, NaN];
    best_p2_overall_recker = [NaN, NaN];
    best_overall_stats_recker = stats_input_recker_segment;
    temp_per_side_diag_handles_recker = [];
    
    fprintf('  Attempting Recker Mod (Global Start Idx %d, Arc: %.2fm, d1: %.2f, Initial d2_base: %.2f):\n', ...
            s_idx_global_start_of_recker_segment, arc_segment_recker, d1_handle_fixed_recker, d2_iterative_base_recker);
    fprintf('    Initial score for this segment (passed in): %.2f [TC:%d, TrC:%d, TkC:%d, TlC:%d]\n', ...
            score_input_recker_segment, stats_input_recker_segment.truck_collision, stats_input_recker_segment.trailer_collision, ...
            stats_input_recker_segment.truck_close, stats_input_recker_segment.trailer_close);
            
    % --- Main Loop: Try Each Direction ---
    for dir_idx_recker = 1:length(dirs_to_try_recker)
        current_direction_recker = dirs_to_try_recker(dir_idx_recker);
        fprintf('    Side (Direction %d):\n', current_direction_recker);
        
        delete(temp_per_side_diag_handles_recker(isgraphics(temp_per_side_diag_handles_recker)));
        temp_per_side_diag_handles_recker = [];
        
        % Adjust d2 growth factor for long curves to prevent overshooting.
        effective_d2_growth_recker = base_d2_growth_recker;
        if arc_segment_recker > LONG_CURVE_ARC_THRESHOLD
            effective_d2_growth_recker = base_d2_growth_recker * D2_GROWTH_ADJUSTMENT_FACTOR_LONG_CURVE;
        end
        effective_d2_growth_recker = max(1.01, effective_d2_growth_recker);
        
        % Run the iterative search for one side.
        [x_try_side, y_try_side, score_side, p1_best_side, p2_best_side, handles_from_side, stats_best_side] = ...
            try_one_side_vc(current_direction_recker, d1_handle_fixed_recker, d2_iterative_base_recker, ...
                            arc_segment_recker, effective_d2_growth_recker, ...
                            x_curve_segment_input, y_curve_segment_input, ...
                            thetaS_segment_input, thetaE_segment_input, map_env, ...
                            truck_original_full_sim, trailer_original_full_sim, ...
                            s_idx_global_start_of_recker_segment, ...
                            g_geom, RO_truck_param, RO_trailer_param, app_costmaps);
                            
        temp_per_side_diag_handles_recker = handles_from_side;
        
        % If this side produced a better result, update the best solution.
        if score_side < bestOverallScore_recker
            bestOverallScore_recker = score_side;
            x_best = x_try_side;
            y_best = y_try_side;
            best_p1_overall_recker = p1_best_side;
            best_p2_overall_recker = p2_best_side;
            best_overall_stats_recker = stats_best_side;
            fprintf('      >> New Best Overall Score for this Curve: %.2f (from Side %d)\n', bestOverallScore_recker, current_direction_recker);
        end
    end
    
    % --- Finalize and Plot Diagnostics ---
    
    delete(temp_per_side_diag_handles_recker(isgraphics(temp_per_side_diag_handles_recker)));
    fprintf('    >> Chosen Final Score for Recker Modification = %.2f\n', bestOverallScore_recker);
    
    modification_succeeded = (best_overall_stats_recker.truck_collision == 0 && ...
                              best_overall_stats_recker.trailer_collision == 0);
                              
    final_diagnostic_handles = [];
    % If a new path was found, plot its control points and handles.
    if ~all(isnan(best_p1_overall_recker)) && ~all(isnan(best_p2_overall_recker)) && ...
       (~isequal(x_best, x_curve_segment_input) || ~isequal(y_best, y_curve_segment_input))
       
        xS_plot_final = x_curve_segment_input(1);
        yS_plot_final = y_curve_segment_input(1);
        xE_plot_final = x_curve_segment_input(end);
        yE_plot_final = y_curve_segment_input(end);
        
        h_cp1_plot = plot(best_p1_overall_recker(1), best_p1_overall_recker(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'DisplayName', 'Chosen P1');
        h_cp2_plot = plot(best_p2_overall_recker(1), best_p2_overall_recker(2), 'co', 'MarkerFaceColor', 'c', 'MarkerSize', 8, 'DisplayName', 'Chosen P2');
        text(best_p1_overall_recker(1), best_p1_overall_recker(2), ' P1', 'Color', 'g', 'VerticalAlignment', 'bottom', 'FontWeight', 'bold');
        text(best_p2_overall_recker(1), best_p2_overall_recker(2), ' P2', 'Color', 'c', 'VerticalAlignment', 'bottom', 'FontWeight', 'bold');
        final_diagnostic_handles = [final_diagnostic_handles, h_cp1_plot, h_cp2_plot];
        
        h_line_S_P1_plot = plot([xS_plot_final, best_p1_overall_recker(1)], [yS_plot_final, best_p1_overall_recker(2)], 'g--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        h_line_E_P2_plot = plot([xE_plot_final, best_p2_overall_recker(1)], [yE_plot_final, best_p2_overall_recker(2)], 'c--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        final_diagnostic_handles = [final_diagnostic_handles, h_line_S_P1_plot, h_line_E_P2_plot];
    end
    
    if isempty(x_best)
        modification_succeeded = false;
    end
    
    drawnow;
    
end % End apply_recker_curve


%% --- Nested Function: try_one_side_vc ---
% Performs the iterative search for a valid Bézier curve on one side of the
% original path. It increases the lateral deviation (d2) until a safe
% curve is found or the maximum number of attempts is reached.
function [x_mod_best_side, y_mod_best_side, best_score_side, ...
          p1_best_side_local, p2_best_side_local, attempt_diag_handles, ...
          stats_best_side_local] = ...
    try_one_side_vc(k_dir_vc, d1_vc, d2_iter_start_vc, arc_seg_vc, d2_growth_vc, ...
                    x_orig_seg_vc, y_orig_seg_vc, thS_vc, thE_vc, ...
                    map_vc, ...
                    truck_full_sim_vc_arg, trailer_full_sim_vc_arg, ...
                    s_idx_global_vc_arg, ...
                    g_geom, RO_truck_param, RO_trailer_param, app_costmaps)

    % --- Local Constants ---
    MAX_ATTEMPTS_PER_SIDE_LOCAL = 16;
    COLLISION_PENALTY_ABSOLUTE_LOCAL = 1e7;
    LOOP_PENALTY_LOCAL = 1000;
    OUTER_WALL_HIT_PENALTY_LOCAL = 7;
    
    d2_current_iter_vc = d2_iter_start_vc;
    xS_vc = x_orig_seg_vc(1);   yS_vc = y_orig_seg_vc(1);
    xE_vc = x_orig_seg_vc(end); yE_vc = y_orig_seg_vc(end);
    
    % --- Initializations ---
    best_score_side = inf;
    x_mod_best_side = x_orig_seg_vc;
    y_mod_best_side = y_orig_seg_vc;
    p1_best_side_local = [NaN, NaN];
    p2_best_side_local = [NaN, NaN];
    stats_best_side_local = struct('truck_collision', inf, 'trailer_collision', inf, 'truck_close', inf, 'trailer_close', inf);
    attempt_diag_handles = [];
    
    % --- Iterative Attempts Loop ---
    for attempt_vc = 1:MAX_ATTEMPTS_PER_SIDE_LOCAL
        
        % --- Generate Candidate Bézier Curve ---
        d2_eff_vc = d2_current_iter_vc;
        max_d2_vc = max(d1_vc * 1.5, arc_seg_vc * 0.6);
        d2_eff_vc = min(d2_eff_vc, max_d2_vc); % Cap d2 to prevent extreme curves.
        
        % Calculate control points for the Bézier curve.
        [p1_att_vc, p2_att_vc] = control_points(xS_vc, yS_vc, xE_vc, yE_vc, thS_vc, thE_vc, d1_vc, d2_eff_vc, k_dir_vc);
        
        score_att_vc = 0;
        
        % Penalize curves where control point handles cross over.
        handles_crossed_vc = segments_intersect([xS_vc, yS_vc], p1_att_vc, [xE_vc, yE_vc], p2_att_vc);
        if handles_crossed_vc
            score_att_vc = score_att_vc + LOOP_PENALTY_LOCAL;
        end
        
        % Generate the points of the Bézier curve.
        num_bpts_vc = max(30, length(x_orig_seg_vc));
        t_param_vc = linspace(0, 1, num_bpts_vc)';
        x_bez_cand_vc = (1-t_param_vc).^3 * xS_vc + 3*(1-t_param_vc).^2 .* t_param_vc * p1_att_vc(1) + ...
                        3*(1-t_param_vc) .* t_param_vc.^2 * p2_att_vc(1) + t_param_vc.^3 * xE_vc;
        y_bez_cand_vc = (1-t_param_vc).^3 * yS_vc + 3*(1-t_param_vc).^2 .* t_param_vc * p1_att_vc(2) + ...
                        3*(1-t_param_vc) .* t_param_vc.^2 * p2_att_vc(2) + t_param_vc.^3 * yE_vc;
        
        % Resample the curve to match the original point density.
        if length(x_bez_cand_vc) < 2
            x_mod_cand_vc = x_orig_seg_vc;
            y_mod_cand_vc = y_orig_seg_vc;
        else
            [x_mod_cand_vc, y_mod_cand_vc] = resample_bezier_to_N(x_bez_cand_vc, y_bez_cand_vc, length(x_orig_seg_vc));
        end
        
        % --- Simulate and Evaluate the Candidate Curve ---
        
        sim_init_idx_for_trailer = min(s_idx_global_vc_arg, size(trailer_full_sim_vc_arg.x, 1));
        sim_init_idx_for_trailer = max(1, sim_init_idx_for_trailer);
        
        tr_x0_geom_sim_vc  = trailer_full_sim_vc_arg.x(sim_init_idx_for_trailer);
        tr_y0_geom_sim_vc  = trailer_full_sim_vc_arg.y(sim_init_idx_for_trailer);
        tr_th0_geom_sim_vc = trailer_full_sim_vc_arg.theta(sim_init_idx_for_trailer);
        
        [truck_sim_cand_geom, trailer_sim_cand_geom] = simulate_vehicle_from_pose(...
            x_mod_cand_vc, y_mod_cand_vc, ...
            xS_vc, yS_vc, thS_vc, ...
            tr_x0_geom_sim_vc, tr_y0_geom_sim_vc, tr_th0_geom_sim_vc, ...
            g_geom, RO_truck_param, RO_trailer_param);
            
        current_danger_counts_cand = struct('truck_collision', 0, 'trailer_collision', 0, 'truck_close', 0, 'trailer_close', 0);
        if isempty(x_mod_cand_vc) || isempty(truck_sim_cand_geom.x) || isempty(trailer_sim_cand_geom.x)
            current_danger_counts_cand.truck_collision = 100; % Penalize simulation failure
            current_danger_counts_cand.trailer_collision = 100;
        else
            [~, current_danger_counts_cand] = evaluate_segment_safety_detailed(...
                truck_sim_cand_geom, trailer_sim_cand_geom, ...
                g_geom, RO_truck_param, RO_trailer_param, app_costmaps);
        end
        stats_cand_vc = current_danger_counts_cand;
        
        % --- Score the Candidate Curve ---
        
        if stats_cand_vc.truck_collision > 0 || stats_cand_vc.trailer_collision > 0
            score_att_vc = score_att_vc + COLLISION_PENALTY_ABSOLUTE_LOCAL * ...
                           (stats_cand_vc.truck_collision + stats_cand_vc.trailer_collision);
        end
        score_att_vc = score_att_vc + (stats_cand_vc.truck_close + stats_cand_vc.trailer_close);
        
        if ~isempty(x_bez_cand_vc) && getOccupancy(map_vc, midpoint_of_curve(x_bez_cand_vc, y_bez_cand_vc)) == 1
            score_att_vc = score_att_vc + OUTER_WALL_HIT_PENALTY_LOCAL;
        end
        
        % --- Logging and Selection ---
        
        is_problematic_log_vc = stats_cand_vc.truck_collision > 0 || stats_cand_vc.trailer_collision > 0 || ...
                                stats_cand_vc.truck_close > 0 || stats_cand_vc.trailer_close > 0 || handles_crossed_vc;
                                
        log_att_vc = (attempt_vc == 1 || attempt_vc == MAX_ATTEMPTS_PER_SIDE_LOCAL || is_problematic_log_vc || score_att_vc < best_score_side);
        
        if log_att_vc
            fprintf('      Att %2d (d1:%.2f,d2i:%.2f,d2e:%.2f): Scr %.2f (LoopP:%d) [TC:%d,TrC:%d,TkC:%d,TlC:%d]\n', ...
                    attempt_vc, d1_vc, d2_current_iter_vc, d2_eff_vc, score_att_vc, handles_crossed_vc * LOOP_PENALTY_LOCAL, ...
                    stats_cand_vc.truck_collision, stats_cand_vc.trailer_collision, ...
                    stats_cand_vc.truck_close, stats_cand_vc.trailer_close);
        end
        
        if is_problematic_log_vc
             h_cand_path_plot_vc = plot(x_mod_cand_vc, y_mod_cand_vc, ':', 'Color', [0.85, 0.85, 0.85], 'LineWidth', 0.5, 'HandleVisibility', 'off');
             attempt_diag_handles = [attempt_diag_handles, h_cand_path_plot_vc];
        end
        if ~isempty(attempt_diag_handles), drawnow limitrate; end
        
        % If this attempt is the best so far, save it.
        if score_att_vc < best_score_side
            if log_att_vc && (attempt_vc > 1 || (attempt_vc == 1 && ~is_problematic_log_vc))
                fprintf('        >> New Best Score for this Side: %.2f\n', score_att_vc);
            end
            best_score_side = score_att_vc;
            x_mod_best_side = x_mod_cand_vc;
            y_mod_best_side = y_mod_cand_vc;
            p1_best_side_local = p1_att_vc;
            p2_best_side_local = p2_att_vc;
            stats_best_side_local = stats_cand_vc;
            
            % If a collision-free solution is found, stop iterating.
            if stats_cand_vc.truck_collision == 0 && stats_cand_vc.trailer_collision == 0 && ~handles_crossed_vc
                fprintf('        Collision-free solution found this side. Score: %.2f. Breaking attempts.\n', score_att_vc);
                break;
            end
        end
        
        % Increase d2 for the next attempt.
        d2_current_iter_vc = d2_current_iter_vc * d2_growth_vc;
        
    end % End of attempts loop for one side.
    
end % End try_one_side_vc


%% --- LOCAL FUNCTION: evaluate_segment_safety_detailed ---
% Checks a segment of a path for both truck and trailer against costmaps.
% Returns a status array for each point and summary counts of dangers.
function [status_array, danger_counts] = evaluate_segment_safety_detailed(...
        truck_path_geom, trailer_path_geom, ...
        g_geom, RO_truck_param, RO_trailer_param, ...
        costmaps_struct)
    
    % --- Input Validation ---
    num_points = 0;
    if isstruct(truck_path_geom) && isfield(truck_path_geom, 'x')
        num_points = length(truck_path_geom.x);
    end
    
    if num_points == 0 || ...
       ~(isstruct(trailer_path_geom) && isfield(trailer_path_geom, 'x') && length(trailer_path_geom.x) == num_points)
        
        status_array = repmat(struct('truck_flag', "safe", 'trailer_flag', "safe", 'is_turning', false, 'location', [NaN NaN]), 0, 1);
        danger_counts = struct('truck_collision', 0, 'truck_close', 0, 'trailer_collision', 0, 'trailer_close', 0);
        return;
    end
    
    % --- Initialization ---
    status_array = repmat(struct('truck_flag', "safe", 'trailer_flag', "safe", 'is_turning', false, 'location', [NaN NaN]), num_points, 1);
    
    % Determine turning status for each point based on angular velocity.
    is_turning_full_truck = false(num_points, 1);
    if num_points > 1 && isfield(truck_path_geom, 'theta') && isfield(g_geom, 'dt') && g_geom.dt > 0
        theta_col = truck_path_geom.theta(:);
        omega_rad_s_truck = [0; diff(unwrap(theta_col))] / g_geom.dt;
        is_turning_full_truck = abs(omega_rad_s_truck) > deg2rad(2); % Threshold for turning
    end
    
    count_tc = 0; count_tn = 0; count_rc = 0; count_rn = 0;
    
    % --- Loop Through Each Point in the Segment ---
    for i_pt = 1:num_points
        % --- Check Truck Safety ---
        [truck_x_ra, truck_y_ra, truck_theta_deg] = convertToRearAxlePose(...
            truck_path_geom.x(i_pt), truck_path_geom.y(i_pt), truck_path_geom.theta(i_pt), ...
            g_geom.Ltr, RO_truck_param);
        truck_pose_ra_deg = [truck_x_ra, truck_y_ra, truck_theta_deg];
        
        isTruckHard = false;
        if ~isempty(costmaps_struct.truck_hard) && isa(costmaps_struct.truck_hard, 'vehicleCostmap')
            isTruckHard = checkOccupied(costmaps_struct.truck_hard, truck_pose_ra_deg);
        end
        
        isTruckClose = false;
        if ~isTruckHard && ~isempty(costmaps_struct.truck_close) && isa(costmaps_struct.truck_close, 'vehicleCostmap')
            isTruckClose = checkOccupied(costmaps_struct.truck_close, truck_pose_ra_deg);
        end
        
        status_array(i_pt).truck_flag = local_flag_from_checks_internal(isTruckHard, isTruckClose);
        if isTruckHard, count_tc = count_tc + 1; elseif isTruckClose, count_tn = count_tn + 1; end
        
        % --- Check Trailer Safety ---
        [trailer_x_ra, trailer_y_ra, trailer_theta_deg] = convertToRearAxlePose(...
            trailer_path_geom.x(i_pt), trailer_path_geom.y(i_pt), trailer_path_geom.theta(i_pt), ...
            g_geom.Ltl, RO_trailer_param);
        trailer_pose_ra_deg = [trailer_x_ra, trailer_y_ra, trailer_theta_deg];
        
        isTrailerHard = false;
        if ~isempty(costmaps_struct.trailer_hard) && isa(costmaps_struct.trailer_hard, 'vehicleCostmap')
            isTrailerHard = checkOccupied(costmaps_struct.trailer_hard, trailer_pose_ra_deg);
        end
        
        isTrailerClose = false;
        if ~isTrailerHard && ~isempty(costmaps_struct.trailer_close) && isa(costmaps_struct.trailer_close, 'vehicleCostmap')
            isTrailerClose = checkOccupied(costmaps_struct.trailer_close, trailer_pose_ra_deg);
        end
        
        status_array(i_pt).trailer_flag = local_flag_from_checks_internal(isTrailerHard, isTrailerClose);
        if isTrailerHard, count_rc = count_rc + 1; elseif isTrailerClose, count_rn = count_rn + 1; end
        
        % --- Store Additional Status Info ---
        status_array(i_pt).is_turning = is_turning_full_truck(i_pt);
        status_array(i_pt).location = [truck_path_geom.x(i_pt), truck_path_geom.y(i_pt)];
    end
    
    danger_counts = struct('truck_collision', count_tc, 'truck_close', count_tn, ...
                           'trailer_collision', count_rc, 'trailer_close', count_rn);
end


%% --- LOCAL HELPER: local_flag_from_checks_internal ---
% Converts boolean safety checks into a standardized string flag.
function flag_str = local_flag_from_checks_internal(is_hard, is_close_not_hard)
    if is_hard
        flag_str = "collision";
    elseif is_close_not_hard
        flag_str = "close";
    else
        flag_str = "safe";
    end
end


%% --- LOCAL HELPER: convertToRearAxlePose ---
% Converts a pose from the vehicle's geometric center to its rear axle center.
% This is required for `vehicleCostmap` checks.
function [x_ra, y_ra, theta_deg] = convertToRearAxlePose(x_geom, y_geom, theta_rad, L_vehicle, RO_vehicle_from_rear_axle)
    offset_geometric_center_to_rear_axle = (L_vehicle / 2) - RO_vehicle_from_rear_axle;
    x_ra = x_geom - offset_geometric_center_to_rear_axle * cos(theta_rad);
    y_ra = y_geom - offset_geometric_center_to_rear_axle * sin(theta_rad);
    theta_deg = rad2deg(theta_rad);
end


%% --- LOCAL UTILITY: simulate_vehicle_from_pose ---
% Simulates the truck and trailer motion along a given path using a Stanley
% controller model, starting from a specified initial state.
function [truck_s, trailer_s] = simulate_vehicle_from_pose(...
    xr_s, yr_s, ...
    x0_truck_geom, y0_truck_geom, th0_truck_geom_rad, ...
    x0_trailer_geom, y0_trailer_geom, th0_trailer_geom_rad, ...
    g_geom, RO_truck_param, RO_trailer_param)
    
    n_s = length(xr_s);
    if n_s == 0
        truck_s.x = []; truck_s.y = []; truck_s.theta = [];
        trailer_s.x = []; trailer_s.y = []; trailer_s.theta = [];
        return;
    end
    
    % Pre-allocate memory for speed.
    truck_s = struct('x', zeros(n_s, 1), 'y', zeros(n_s, 1), 'theta', zeros(n_s, 1));
    trailer_s = struct('x', zeros(n_s, 1), 'y', zeros(n_s, 1), 'theta', zeros(n_s, 1));
    
    % Set initial conditions.
    truck_s.x(1) = x0_truck_geom;
    truck_s.y(1) = y0_truck_geom;
    truck_s.theta(1) = th0_truck_geom_rad;
    
    trailer_s.x(1) = x0_trailer_geom;
    trailer_s.y(1) = y0_trailer_geom;
    trailer_s.theta(1) = th0_trailer_geom_rad;
    
    if n_s < 2, return; end
    
    if ~isfield(g_geom, 'WB_truck')
        error('simulate_vehicle_from_pose: g_geom.WB_truck (truck wheelbase) is missing.');
    end
    
    % --- Simulation Loop ---
    for i_s = 1:n_s-1
        current_truck_geom_x = truck_s.x(i_s);
        current_truck_geom_y = truck_s.y(i_s);
        current_truck_geom_theta = truck_s.theta(i_s);
        
        % Stanley Controller Logic
        [~, idx_cl_s] = min((xr_s - current_truck_geom_x).^2 + (yr_s - current_truck_geom_y).^2);
        cur_srch_s = idx_cl_s;
        tgt_idx_s = idx_cl_s;
        
        while cur_srch_s <= n_s
            if (xr_s(cur_srch_s) - current_truck_geom_x)^2 + (yr_s(cur_srch_s) - current_truck_geom_y)^2 >= g_geom.look^2
                tgt_idx_s = cur_srch_s; break;
            end
            if cur_srch_s == n_s, tgt_idx_s = n_s; break; end
            cur_srch_s = cur_srch_s + 1;
        end
        
        xt_s = xr_s(tgt_idx_s); yt_s = yr_s(tgt_idx_s);
        alp_s = wrapToPi(atan2(yt_s - current_truck_geom_y, xt_s - current_truck_geom_x) - current_truck_geom_theta);
        Le_s = max(sqrt((xt_s - current_truck_geom_x)^2 + (yt_s - current_truck_geom_y)^2), 0.1);
        del_s = atan2(2 * g_geom.WB_truck * sin(alp_s), Le_s);
        del_s = max(-deg2rad(40), min(deg2rad(40), del_s)); % Clamp steering angle
        
        % Update Truck State
        vx_truck_geom = g_geom.v * cos(current_truck_geom_theta);
        vy_truck_geom = g_geom.v * sin(current_truck_geom_theta);
        omega_truck = (g_geom.v / g_geom.WB_truck) * tan(del_s);
        
        truck_s.x(i_s+1) = current_truck_geom_x + vx_truck_geom * g_geom.dt;
        truck_s.y(i_s+1) = current_truck_geom_y + vy_truck_geom * g_geom.dt;
        truck_s.theta(i_s+1) = wrapToPi(current_truck_geom_theta + omega_truck * g_geom.dt);
        
        % Update Trailer State based on new truck position
        hitch_x = truck_s.x(i_s+1) - (g_geom.Ltr / 2) * cos(truck_s.theta(i_s+1));
        hitch_y = truck_s.y(i_s+1) - (g_geom.Ltr / 2) * sin(truck_s.theta(i_s+1));
        current_trailer_geom_x = trailer_s.x(i_s);
        current_trailer_geom_y = trailer_s.y(i_s);
        
        trailer_s.theta(i_s+1) = wrapToPi(atan2(hitch_y - current_trailer_geom_y, hitch_x - current_trailer_geom_x));
        
        dist_hitch_to_trailer_geom_center = g_geom.Lh + (g_geom.Ltl / 2);
        
        trailer_s.x(i_s+1) = hitch_x - dist_hitch_to_trailer_geom_center * cos(trailer_s.theta(i_s+1));
        trailer_s.y(i_s+1) = hitch_y - dist_hitch_to_trailer_geom_center * sin(trailer_s.theta(i_s+1));
    end
end


%% --- LOCAL UTILITY: control_points ---
% Calculates Bézier control points P1 and P2.
function [p1, p2] = control_points(xS, yS, xE, yE, thetaS, thetaE, d1, d2, k_dir)
    % Inline functions for perpendicular vectors.
    right_perp = @(th) [sin(th), -cos(th)];
    left_perp  = @(th) [-sin(th), cos(th)];
    
    if k_dir > 0 % Left side modification
        perpS = left_perp(thetaS);
        perpE = left_perp(thetaE);
    else % Right side modification
        perpS = right_perp(thetaS);
        perpE = right_perp(thetaE);
    end
    
    p1 = [xS, yS] + d1 * [cos(thetaS), sin(thetaS)] + d2 * perpS;
    p2 = [xE, yE] - d1 * [cos(thetaE), sin(thetaE)] + d2 * perpE;
end


%% --- LOCAL UTILITY: resample_bezier_to_N ---
% Resamples a curve defined by points (xb, yb) to have N equally spaced points.
function [xo, yo] = resample_bezier_to_N(xb, yb, N)
    if isempty(xb) || length(xb) < 2, xo = xb; yo = yb; return; end
    if N <= 0
        xo = []; yo = [];
        if N == 1 && ~isempty(xb), xo = xb(1); yo = yb(1); end
        return;
    end
    
    % Calculate cumulative arc length.
    al = [0; cumsum(sqrt(diff(xb(:)).^2 + diff(yb(:)).^2))];
    tl = al(end);
    
    if tl < 1e-6 % If the curve has no length, replicate the first point.
        xo = repmat(xb(1), N, 1);
        yo = repmat(yb(1), N, 1);
        return;
    end
    
    % Interpolate to get N points.
    st = linspace(0, tl, N);
    xo = interp1(al, xb, st, 'linear')';
    yo = interp1(al, yb, st, 'linear')';
    xo = xo(:); % Ensure column vector output
    yo = yo(:);
end


%% --- LOCAL UTILITY: segments_intersect ---
% Checks if two line segments, [p1s1, p2s1] and [p1s2, p2s2], intersect.
function tf = segments_intersect(p1s1, p2s1, p1s2, p2s2)
    % Helper function to determine orientation (collinear, clockwise, or counter-clockwise).
    function o = orientation(p, q, r)
        val = (q(2) - p(2)) * (r(1) - q(1)) - (q(1) - p(1)) * (r(2) - q(2));
        if abs(val) < 1e-9, o = 0; % Collinear
        elseif val > 0, o = 1;      % Clockwise
        else, o = 2;                % Counter-clockwise
        end
    end
    
    % Helper function to check if point q lies on segment pr.
    function tf_on = on_segment(p, q, r)
        tf_on = (q(1) <= max(p(1), r(1)) && q(1) >= min(p(1), r(1)) && ...
                 q(2) <= max(p(2), r(2)) && q(2) >= min(p(2), r(2)));
    end
    
    % Find the four orientations needed to check for general and special cases.
    o1 = orientation(p1s1, p2s1, p1s2);
    o2 = orientation(p1s1, p2s1, p2s2);
    o3 = orientation(p1s2, p2s2, p1s1);
    o4 = orientation(p1s2, p2s2, p2s1);
    
    % General case: The segments cross each other.
    if o1 ~= 0 && o2 ~= 0 && o3 ~= 0 && o4 ~= 0
        if o1 ~= o2 && o3 ~= o4
            tf = true; return;
        end
    % Special Cases: The segments are collinear and overlap.
    elseif o1 == 0 && on_segment(p1s1, p1s2, p2s1), tf = true; return;
    elseif o2 == 0 && on_segment(p1s1, p2s2, p2s1), tf = true; return;
    elseif o3 == 0 && on_segment(p1s2, p1s1, p2s2), tf = true; return;
    elseif o4 == 0 && on_segment(p1s2, p2s1, p2s2), tf = true; return;
    end
    
    tf = false;
end


%% --- LOCAL UTILITY: midpoint_of_curve ---
% Finds the geometric midpoint of a curve defined by points.
function pt_mid = midpoint_of_curve(xb_mid, yb_mid)
    if isempty(xb_mid), pt_mid = [NaN, NaN]; return; end
    mid_idx = round(numel(xb_mid) / 2);
    if mid_idx < 1, mid_idx = 1; end
    if mid_idx > numel(xb_mid), mid_idx = numel(xb_mid); end
    pt_mid = [xb_mid(mid_idx), yb_mid(mid_idx)];
end


%% --- LOCAL UTILITY: plotVehicleFootprint ---
% Manually plots a vehicle's rectangular footprint given its dimensions and pose.
function plotVehicleFootprint(vehicle_dims, pose_ra_deg, plot_color, display_name_str, parent_axes)
    % ARGS:
    % vehicle_dims:     A vehicleDimensions object or a struct with L, W, RearOverhang
    % pose_ra_deg:      [x_rear_axle, y_rear_axle, theta_degrees]
    % plot_color:       e.g., 'b' or [0 0 1]
    % display_name_str: String for the legend
    % parent_axes:      The axes to plot on
    
    L = vehicle_dims.Length;
    W = vehicle_dims.Width;
    
    % Calculate distances from the rear axle reference point.
    dist_ra_to_front = L - vehicle_dims.RearOverhang;
    dist_ra_to_rear = -vehicle_dims.RearOverhang;
    
    x_ra = pose_ra_deg(1);
    y_ra = pose_ra_deg(2);
    theta_rad = deg2rad(pose_ra_deg(3));
    
    % Define corners in the vehicle's reference frame (rear axle at origin).
    corners_vehicle_frame = [
        dist_ra_to_front,  W/2;  % Front-right
        dist_ra_to_front, -W/2;  % Front-left
        dist_ra_to_rear,  -W/2;  % Rear-left
        dist_ra_to_rear,   W/2;  % Rear-right
        dist_ra_to_front,  W/2;  % Close the polygon
    ];
    
    % Create rotation matrix.
    R = [cos(theta_rad), -sin(theta_rad);
         sin(theta_rad),  cos(theta_rad)];
    
    % Rotate and translate corners to the world frame.
    corners_world_frame = (R * corners_vehicle_frame')';
    corners_world_frame(:, 1) = corners_world_frame(:, 1) + x_ra;
    corners_world_frame(:, 2) = corners_world_frame(:, 2) + y_ra;
    
    % Handle optional arguments.
    if nargin < 5 || isempty(parent_axes), parent_axes = gca; end
    if nargin < 4, display_name_str = 'Vehicle Footprint'; end
    if nargin < 3, plot_color = 'b'; end
    
    % Plot the footprint.
    plot(parent_axes, corners_world_frame(:, 1), corners_world_frame(:, 2), ...
         'Color', plot_color, 'LineWidth', 1.5, 'DisplayName', display_name_str);
end