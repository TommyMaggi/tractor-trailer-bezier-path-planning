%% ========== Curve-by-curve modifier (Enhanced with Trailer-Lag Window & Diagnostics) ==========
function [x_ref_mod,y_ref_mod, overall_modification_successful] = modify_curves_bezier(x_ref,y_ref,theta_ref,...
                                      curve_bounds_padded,map,truck,trailer,~,~, danger_original_segment_optional)
    % Modifies a reference path curve by curve to ensure collision-free
    % traversal for a tractor-trailer system.
    % v5.14: - Success based ONLY on NO HARD COLLISIONS. Clearance logic removed from scoring.
    %        - Initial d2 value and d2 growth factor are scaled based on initial risk 
    %          (hard collisions vs. only "close" calls).
    %        - d2_iterative_base capped for very low kappa values.
    %        - D2 growth factor for subsequent attempts further adjusted for long curves.
    %        - If all Recker modifications are worse (introduce collisions or more risk) 
    %          than original segment, original is kept.
    %        - Corrected try_one_side function call/definition arguments.
    % Inputs:
    %   x_ref, y_ref, theta_ref: Original reference path coordinates and orientation.
    %   curve_bounds_padded: Nx2 matrix defining start and end indices of curves.
    %   map: Occupancy map (binaryOccupancyMap object).
    %   truck, trailer: Structs containing the *full original simulated path* of truck and trailer.
    %   danger_original_segment_optional: (Optional) Struct from initial eval by the calling function.
    % Outputs:
    %   x_ref_mod, y_ref_mod: Modified path coordinates.
    %   overall_modification_successful: Boolean flag, true if all modifications were successful.

    g = geom(); % Get geometry and control constants

    if nargin < 10 || isempty(danger_original_segment_optional) 
        danger_original_segment_optional = struct('truck_collision',0,'truck_close',0,'trailer_collision',0,'trailer_close',0); 
    end

    x_ref = x_ref(:); y_ref = y_ref(:); 
    theta_ref = unwrap(theta_ref(:)); 
    x_ref_mod = []; y_ref_mod = []; 
    last_idx  = 1; 
    overall_modification_successful = true; 

    figure_handle = figure('Name','Recker Curve Evaluation (Enhanced Diagnostics)');
    show(map); hold on;
    plot(x_ref,y_ref,'k--','DisplayName','Original Path');
    h_initial_truck_collision = plot(NaN,NaN,'rx','MarkerSize',10,'LineWidth',2,'DisplayName','Initial Truck Collision');
    h_initial_trailer_collision = plot(NaN,NaN,'mx','MarkerSize',10,'LineWidth',2,'DisplayName','Initial Trailer Collision');
    axis equal; grid on; title('Path Modification Diagnostics');
    legend_handles = [findobj(gca,'DisplayName','Original Path'), h_initial_truck_collision, h_initial_trailer_collision]; 
    
    trailer_lag_distance = g.Lh + g.Ltl; 
    distance_per_sample_step = g.v * g.dt; 
    if distance_per_sample_step <= 1e-6 
        num_lag_samples = 0;
        warning('geom:lag_calc','Distance per sample step is very small or zero; num_lag_samples set to 0.');
    else
        num_lag_samples = ceil(trailer_lag_distance / distance_per_sample_step);
    end
    fprintf('Trailer lag samples: %d (based on %.2fm lag distance and %.3fm/sample)\n', num_lag_samples, trailer_lag_distance, distance_per_sample_step);
    
    current_curve_final_diag_handles = [];
    for c = 1:size(curve_bounds_padded,1)
        
        if ~isempty(current_curve_final_diag_handles) && all(isgraphics(current_curve_final_diag_handles))
            delete(current_curve_final_diag_handles);
        end
        current_curve_final_diag_handles = []; 
        fprintf('\n--- CURVE %d of %d ---\n', c, size(curve_bounds_padded,1));
        s_idx_current_curve = curve_bounds_padded(c,1);
        e_idx_current_curve = curve_bounds_padded(c,2);
        
        if s_idx_current_curve >= e_idx_current_curve || s_idx_current_curve < 1 || e_idx_current_curve > length(x_ref)
            fprintf('  Skipping invalid nominal curve segment %d-%d.\n', s_idx_current_curve, e_idx_current_curve);
            if last_idx <= e_idx_current_curve && last_idx <= length(x_ref)
                 safe_e_idx = min(e_idx_current_curve, length(x_ref));
                 if last_idx <= safe_e_idx 
                    x_ref_mod = [x_ref_mod; x_ref(last_idx:safe_e_idx)];
                    y_ref_mod = [y_ref_mod; y_ref(last_idx:safe_e_idx)];
                    last_idx = safe_e_idx + 1;
                 elseif last_idx > safe_e_idx && safe_e_idx >= curve_bounds_padded(c,1) 
                    last_idx = safe_e_idx + 1; 
                 end
            end
            continue;
        end
        
        x_curve_nominal = x_ref(s_idx_current_curve:e_idx_current_curve);
        y_curve_nominal = y_ref(s_idx_current_curve:e_idx_current_curve);
        s_idx_recker_input = s_idx_current_curve;
        e_idx_recker_input = e_idx_current_curve;
        
        current_arc_len = sum(hypot(diff(x_curve_nominal),diff(y_curve_nominal)));
        if current_arc_len < 1.2*(g.Ltr+g.Ltl) && s_idx_recker_input > 5 && e_idx_recker_input < length(x_ref)-5
            s_idx_recker_input_orig = s_idx_recker_input; e_idx_recker_input_orig = e_idx_recker_input;
            s_idx_recker_input = s_idx_recker_input - 5;
            e_idx_recker_input = e_idx_recker_input + 5;
            fprintf('  Nominal: %d-%d. Tiny-corner extension: %d-%d -> %d-%d.\n', s_idx_current_curve, e_idx_current_curve, s_idx_recker_input_orig, e_idx_recker_input_orig, s_idx_recker_input, e_idx_recker_input);
        else
            fprintf('  Nominal: %d-%d. Recker Input: %d-%d.\n', s_idx_current_curve, e_idx_current_curve, s_idx_recker_input, e_idx_recker_input);
        end
        x_segment_for_recker = x_ref(s_idx_recker_input:e_idx_recker_input);
        y_segment_for_recker = y_ref(s_idx_recker_input:e_idx_recker_input);
        h_nom_curve = plot(x_curve_nominal, y_curve_nominal,'m-','LineWidth',1.5, 'DisplayName', sprintf('C%d Nom.', c));
        text(x_curve_nominal(round(end/2)), y_curve_nominal(round(end/2)), sprintf('%d',c),...
            'Color','m','FontWeight','bold', 'VerticalAlignment','bottom', 'HorizontalAlignment', 'center');
        if ~any(strcmp(get(legend_handles,'DisplayName'), get(h_nom_curve,'DisplayName')))
             legend_handles(end+1) = h_nom_curve;
        end
        eval_s_idx = s_idx_current_curve;
        temp_eval_e_idx = e_idx_current_curve + num_lag_samples;
        eval_e_idx = min(temp_eval_e_idx, length(x_ref));
        if c < size(curve_bounds_padded,1)
            s_idx_next_curve = curve_bounds_padded(c+1,1);
            if s_idx_next_curve > eval_s_idx && s_idx_next_curve <= length(x_ref)
                eval_e_idx = min(eval_e_idx, s_idx_next_curve - 1);
            end
        end
        eval_e_idx = max(eval_e_idx, eval_s_idx); 
        fprintf('  Initial Safety Eval Window (orig. path): %d-%d.\n', eval_s_idx, eval_e_idx);
        needs_modification = false;
        initial_collision_points_truck_x = []; initial_collision_points_truck_y = [];
        initial_collision_points_trailer_x = []; initial_collision_points_trailer_y = [];
        danger_original_segment_for_this_curve = struct('truck_collision',0,'truck_close',0,'trailer_collision',0,'trailer_close',0); 

        if eval_s_idx < eval_e_idx 
            current_eval_s_idx_clamped = min(eval_s_idx, size(truck.x,1));
            current_eval_e_idx_clamped = min(eval_e_idx, size(truck.x,1)); 
            current_eval_s_idx_clamped = min(current_eval_s_idx_clamped, size(trailer.x,1));
            current_eval_e_idx_clamped = min(current_eval_e_idx_clamped, size(trailer.x,1));
            if current_eval_s_idx_clamped < current_eval_e_idx_clamped
                fprintf('  Evaluating original path segment from global index %d to %d.\n', current_eval_s_idx_clamped, current_eval_e_idx_clamped);
                [status_original_segment, danger_original_segment_for_this_curve] = evaluate_trajectory_safety(truck,trailer,map,current_eval_s_idx_clamped,current_eval_e_idx_clamped);
                
                truck_flags_original   = string({status_original_segment.truck_flag});
                trailer_flags_original = string({status_original_segment.trailer_flag});
                
                if isfield(status_original_segment, 'is_turning') && ~isempty(status_original_segment)
                    turning_flags_original = [status_original_segment.is_turning];
                else
                    turning_flags_original = false(1, length(status_original_segment)); 
                end
                
                needs_modification = any(truck_flags_original=="collision"|trailer_flags_original=="collision") || ...
                                     any(turning_flags_original & (truck_flags_original=="close"|trailer_flags_original=="close"));
                
                fprintf('    Initial Status: Needs Mod = %s (TC:%d, TrC:%d, TkC:%d, TlC:%d)\n', ...
                        matlab.lang.makeValidName(string(needs_modification)), ... 
                        danger_original_segment_for_this_curve.truck_collision, danger_original_segment_for_this_curve.trailer_collision, ...
                        danger_original_segment_for_this_curve.truck_close, danger_original_segment_for_this_curve.trailer_close);

                for ii = 1:length(status_original_segment)
                    k_global = current_eval_s_idx_clamped + ii - 1; 
                    if status_original_segment(ii).truck_flag == "collision"
                        initial_collision_points_truck_x(end+1) = truck.x(k_global);
                        initial_collision_points_truck_y(end+1) = truck.y(k_global);
                    end
                    if status_original_segment(ii).trailer_flag == "collision"
                        initial_collision_points_trailer_x(end+1) = trailer.x(k_global); 
                        initial_collision_points_trailer_y(end+1) = trailer.y(k_global);
                    end
                end
            else
                 fprintf('  Warning: Clamped initial eval window %d-%d too small. Assuming safe.\n', current_eval_s_idx_clamped, current_eval_e_idx_clamped);
            end
        else
            fprintf('  Warning: Base initial eval window %d-%d too small. Assuming safe.\n', eval_s_idx, eval_e_idx);
        end
        
        set(h_initial_truck_collision, 'XData', initial_collision_points_truck_x, 'YData', initial_collision_points_truck_y);
        set(h_initial_trailer_collision, 'XData', initial_collision_points_trailer_x, 'YData', initial_collision_points_trailer_y);
        
        current_segment_modification_successful = true; 

        if needs_modification
            fprintf('  🛑 Status: Unsafe. Applying Recker to segment %d-%d.\n', s_idx_recker_input, e_idx_recker_input);
            
            [x_mod,y_mod, diag_handles_from_recker, current_segment_modification_successful] = apply_recker_curve(x_segment_for_recker,y_segment_for_recker,...
                theta_ref(s_idx_recker_input),theta_ref(e_idx_recker_input),map,truck,trailer, s_idx_recker_input, danger_original_segment_for_this_curve); 
            
            if ~current_segment_modification_successful
                fprintf('    ⚠️ WARNING: Recker modification for curve %d (segment %d-%d) FAILED to find a collision-free path.\n', c, s_idx_recker_input, e_idx_recker_input);
                overall_modification_successful = false; 
            else
                fprintf('    ✅ Recker modification for curve %d (segment %d-%d) SUCCEEDED in finding a collision-free path.\n', c, s_idx_recker_input, e_idx_recker_input);
            end

            current_curve_final_diag_handles = diag_handles_from_recker; 
            
            h_mod_curve = plot(x_mod,y_mod,'g-','LineWidth',2,'DisplayName',sprintf('C%d Mod.',c));
            if ~any(strcmp(get(legend_handles,'DisplayName'), get(h_mod_curve,'DisplayName')))
                legend_handles(end+1) = h_mod_curve;
            end
        else
            fprintf('  ✅ Status: Safe. Using segment %d-%d as is.\n', s_idx_recker_input, e_idx_recker_input);
            x_mod = x_segment_for_recker;
            y_mod = y_segment_for_recker;
        end
        fprintf('    Output segment (global %d-%d) has %d points.\n',...
            s_idx_recker_input, e_idx_recker_input, length(x_mod));
        
        if last_idx < s_idx_recker_input
            gap_s = last_idx;
            gap_e = s_idx_recker_input-1;
            if gap_s <= gap_e && gap_s > 0 && gap_e <= length(x_ref)
                x_ref_mod = [x_ref_mod; x_ref(gap_s:gap_e)];
                y_ref_mod = [y_ref_mod; y_ref(gap_s:gap_e)];
            elseif gap_s > gap_e
            end
        end
        
        if ~isempty(x_mod)
            x_ref_mod = [x_ref_mod; x_mod(:)];
            y_ref_mod = [y_ref_mod; y_mod(:)];
        else
            warning('geom:empty_mod','x_mod for curve %d was empty. Stitching original Recker input segment.', c);
            x_ref_mod = [x_ref_mod; x_segment_for_recker(:)];
            y_ref_mod = [y_ref_mod; y_segment_for_recker(:)];
             overall_modification_successful = false; 
        end
        last_idx  = e_idx_recker_input+1;
        drawnow; 
    end 
    if last_idx <= length(x_ref)
        x_ref_mod = [x_ref_mod; x_ref(last_idx:end)];
        y_ref_mod = [y_ref_mod; y_ref(last_idx:end)];
    end
    h_final_path = plot(x_ref_mod,y_ref_mod,'b-','LineWidth',2,'DisplayName','Final Path');
    if ~any(strcmp(get(legend_handles,'DisplayName'), get(h_final_path,'DisplayName')))
        legend_handles(end+1) = h_final_path;
    end
    
    try 
        valid_legend_handles = legend_handles(isgraphics(legend_handles));
        [~,ia,~] = unique(cellstr(get(valid_legend_handles,'DisplayName')), 'stable');
        legend(valid_legend_handles(ia), 'Location', 'bestoutside');
    catch ME
        warning('Could not update legend: %s', ME.message);
    end
    hold off;
    fprintf('\n--- Path Modification Complete ---\n');
    if ~overall_modification_successful
        fprintf('--- OVERALL PATH MODIFICATION FAILED TO RESOLVE ALL ISSUES. ---\n');
    else
        fprintf('--- OVERALL PATH MODIFICATION SUCCEEDED. ---\n');
    end
end

%% ========== Polygon Collision Checker (More Robust) ==========
function is_colliding = check_polygon_collision(poly_vertices, map)
    if isempty(poly_vertices) || size(poly_vertices,1) < 3
        is_colliding = false; 
        return;
    end
    map_resolution = map.Resolution; 
    OCCUPIED_VALUE = 1; 
    occupancy_at_vertices = getOccupancy(map, poly_vertices);
    if any(occupancy_at_vertices == OCCUPIED_VALUE) 
        is_colliding = true;
        return;
    end
    edge_sample_points = [];
    num_edges = size(poly_vertices, 1);
    for k_edge = 1:num_edges
        p_start = poly_vertices(k_edge, :);
        p_end = poly_vertices(mod(k_edge, num_edges) + 1, :); 
        edge_length = hypot(p_end(1)-p_start(1), p_end(2)-p_start(2));
        num_points_on_edge = max(5, round(edge_length * map_resolution * 2) + 1); 
        current_edge_samples = interpolate_line(p_start, p_end, num_points_on_edge);
        edge_sample_points = [edge_sample_points; current_edge_samples]; %#ok<AGROW>
    end
    if ~isempty(edge_sample_points)
        edge_sample_points = unique(edge_sample_points, 'rows'); 
        occupancy_at_edge_points = getOccupancy(map, edge_sample_points);
        if any(occupancy_at_edge_points == OCCUPIED_VALUE) 
            is_colliding = true;
            return;
        end
    end
    bounding_box = [min(poly_vertices,[],1); max(poly_vertices,[],1)];
    if diff(bounding_box(:,1)) < (0.1/map_resolution) || diff(bounding_box(:,2)) < (0.1/map_resolution)
        interior_sample_points = [];
    else
        step_size_interior = 0.5 / map_resolution; 
        [Xi, Yi] = meshgrid(bounding_box(1,1) : step_size_interior : bounding_box(2,1), ...
                            bounding_box(1,2) : step_size_interior : bounding_box(2,2));
        if isempty(Xi) 
            interior_sample_points = [];
        else
            are_points_in_polygon = inpolygon(Xi(:), Yi(:), poly_vertices(:,1), poly_vertices(:,2));
            interior_sample_points = [Xi(are_points_in_polygon), Yi(are_points_in_polygon)];
        end
    end
    if ~isempty(interior_sample_points)
        interior_sample_points = unique(interior_sample_points, 'rows');
        occupancy_at_interior_points = getOccupancy(map, interior_sample_points);
        if any(occupancy_at_interior_points == OCCUPIED_VALUE) 
            is_colliding = true;
            return;
        end
    end
    is_colliding = false;
end

%% ========== Safety evaluator (ignores exit‑leg proximity) ==========
function [status,danger] = evaluate_trajectory_safety(truck,trailer,map,idx_s,idx_e)
g = geom(); 
inflated = copy(map); inflate(inflated,0.30); 
path_len_truck = length(truck.x);
path_len_trailer = length(trailer.x); 
idx_s = max(1, idx_s);
idx_e = min(idx_e, path_len_truck);
idx_e = min(idx_e, path_len_trailer); 
if idx_s > idx_e 
    status = struct('truck_flag',{},'trailer_flag',{},'is_turning',{},'location',{});
    danger = struct('truck_collision',0,'truck_close',0,'trailer_collision',0,'trailer_close',0);
    return;
end
idxs = idx_s:idx_e;  
n = length(idxs);
status = repmat(struct('truck_flag',"safe",'trailer_flag',"safe",'is_turning',false,'location',[NaN NaN]), n, 1);
if path_len_truck > 1
    truck_theta_col = truck.theta(1:path_len_truck);
    if size(truck_theta_col,2) > 1, truck_theta_col = truck_theta_col'; end
    omega  = [0; wrapToPi(diff(truck_theta_col))]/g.dt; 
else
    omega = zeros(path_len_truck,1);
end
turning_full_path = abs(omega) > deg2rad(2); 
turning_segment = false(1,n); 
valid_turning_indices = idxs(idxs <= length(turning_full_path));
if ~isempty(valid_turning_indices) 
    turning_segment(idxs <= length(turning_full_path)) = turning_full_path(valid_turning_indices);
end
tc=0; tn=0; rc=0; rn=0;
for ii = 1:n
    k = idxs(ii);
    trkPoly = get_rectangle(truck.x(k),truck.y(k),truck.theta(k),g.Ltr,g.Wtr);
    trlPoly = get_rectangle(trailer.x(k),trailer.y(k),trailer.theta(k),g.Ltl,g.Wtl);
    trkCol = check_polygon_collision(trkPoly,map);
    trlCol = check_polygon_collision(trlPoly,map);
    trkN = false; trlN = false;
    if ~trkCol
        trkN = check_polygon_collision(trkPoly,inflated);
    end
    if ~trlCol
        trlN = check_polygon_collision(trlPoly,inflated);
    end
    current_is_turning = turning_segment(ii);
    if trlN && ~trlCol && ~current_is_turning 
        grad_check_point = trlPoly(3,:); 
        grad = gradient_to_nearest_obstacle(grad_check_point,map);   
        movingAway = dot(grad,[cos(trailer.theta(k)) sin(trailer.theta(k))]) < -0.1; 
        if movingAway, trlN = false; end 
    end
    tc = tc + trkCol; tn = tn + trkN; 
    rc = rc + trlCol; rn = rn + trlN; 
    status(ii).truck_flag   = flag_from_check(trkCol,trkN); 
    status(ii).trailer_flag = flag_from_check(trlCol,trlN);
    status(ii).is_turning   = current_is_turning;
    status(ii).location     = [truck.x(k) truck.y(k)];
end
danger = struct('truck_collision',tc,'truck_close',tn,...
                'trailer_collision',rc,'trailer_close',rn);
end
%% ========== Recker Bézier modifier (More Aggressive for Sharp Turns, Loop Prevention) ==========
function [x_best,y_best, final_diagnostic_handles, modification_succeeded] = apply_recker_curve(x_curve_segment,y_curve_segment,thetaS_segment,thetaE_segment,...
                                            map,truck_original_path,trailer_original_path, s_idx_global_start_of_segment, danger_metrics_original_segment) 
    g = geom(); 
    OUTER_WALL_HIT_PENALTY = 7;    
    LOOP_PENALTY = 1000; 
    COLLISION_PENALTY_ABSOLUTE = 1e7; 

    D1_ARC_LIMIT_FACTOR = 0.5; 
    D1_RECKER_BASE_FACTOR = 0.5; 
    D2_BASE_MIN_FACTOR_OF_D1 = 0.3; 
    MAX_ATTEMPTS_PER_SIDE = 16;    
    
    % Risk-adaptive and length-adaptive d2 parameters
    LONG_CURVE_ARC_THRESHOLD = 4.0; 
    INITIAL_D2_SCALE_FACTOR_ONLY_CLOSE = 0.20;  % Significantly smaller initial d2 if only "close"
    INITIAL_D2_SCALE_FACTOR_COLLISION = 0.8;   % Slightly reduced initial d2 even for collisions
    
    D2_GROWTH_FACTOR_ONLY_CLOSE = 1.05;      % Very gentle d2 growth if only "close"
    D2_GROWTH_FACTOR_COLLISION = 1.18;       % More assertive d2 growth if collisions
    D2_GROWTH_ADJUSTMENT_FACTOR_LONG_CURVE = 0.95; 
    MIN_KAPPA_FOR_D2_FORMULA = 0.05; 
    D2_BASE_FOR_VERY_LOW_KAPPA = 0.10; % Further reduced base for very low kappa


    Ltot = g.Ltr + g.Lh + g.Ltl;
    arc_segment  = sum(hypot(diff(x_curve_segment),diff(y_curve_segment)));
    delta_angle_segment = abs(wrapToPi(thetaE_segment-thetaS_segment));

    d1_recker_component = D1_RECKER_BASE_FACTOR * Ltot;
    d1_arc_component = D1_ARC_LIMIT_FACTOR * arc_segment;
    
    if arc_segment < 1e-2 
        warning('apply_recker_curve:short_arc','Input curve segment arc length is very small (%.4f m).', arc_segment);
        d1_handle_fixed = min(d1_recker_component, g.Ltr); 
        fprintf('      Info: Short arc (%.2fm). Using d1_fixed: %.2f (min of Recker base and Ltr).\n', arc_segment, d1_handle_fixed);
        kappa_segment = 2; 
    else
        d1_handle_fixed = min(d1_recker_component, d1_arc_component);
        kappa_segment = delta_angle_segment/arc_segment;
    end
    d1_handle_fixed = max(d1_handle_fixed, 0.1 * Ltot); 

    Rmin_approx  = (g.Lh+g.Ltl)/tan(deg2rad(35)); 
    
    if kappa_segment < MIN_KAPPA_FOR_D2_FORMULA 
        d2_iterative_base = D2_BASE_FOR_VERY_LOW_KAPPA;
        fprintf('      Info: Low kappa (%.3f < %.3f). Setting d2_iterative_base to default low kappa value: %.2f\n', kappa_segment, MIN_KAPPA_FOR_D2_FORMULA, d2_iterative_base);
    else 
        d2_iterative_base = max(0.15, 0.8*(1/kappa_segment - Rmin_approx)); % Original base for normal kappa
        if kappa_segment > 0.4 || arc_segment < 1.5 * d1_handle_fixed 
            min_d2_boosted = D2_BASE_MIN_FACTOR_OF_D1 * d1_handle_fixed;
            if d2_iterative_base < min_d2_boosted
                fprintf('      Info: d2_iterative_base boosted for sharp/short curve from %.2f to %.2f.\n', d2_iterative_base, min_d2_boosted);
                d2_iterative_base = min_d2_boosted;
            end
        end
    end
    
    % Determine initial d2 scaling and base D2_GROWTH_FACTOR based on initial risk
    base_d2_growth_for_attempts = D2_GROWTH_FACTOR_COLLISION; % Default
    initial_d2_scale_factor_to_apply = INITIAL_D2_SCALE_FACTOR_COLLISION; % Default

    if nargin > 8 && ~isempty(danger_metrics_original_segment) 
        is_initial_collision = (danger_metrics_original_segment.truck_collision > 0 || danger_metrics_original_segment.trailer_collision > 0);
        sum_close_calls = danger_metrics_original_segment.truck_close + danger_metrics_original_segment.trailer_close;
        
        if ~is_initial_collision && sum_close_calls > 0 
            initial_d2_scale_factor_to_apply = INITIAL_D2_SCALE_FACTOR_ONLY_CLOSE;
            base_d2_growth_for_attempts = D2_GROWTH_FACTOR_ONLY_CLOSE;
            fprintf('      Info: Initial state is "close calls" only (sum_close=%d). Using D2_SCALE_FACTOR=%.2f, D2_GROWTH_FACTOR=%.2f for attempts.\n', sum_close_calls, initial_d2_scale_factor_to_apply, base_d2_growth_for_attempts);
        elseif is_initial_collision 
             base_d2_growth_for_attempts = D2_GROWTH_FACTOR_COLLISION;
             initial_d2_scale_factor_to_apply = INITIAL_D2_SCALE_FACTOR_COLLISION; 
             fprintf('      Info: Initial state has collisions. Using D2_SCALE_FACTOR=%.2f, D2_GROWTH_FACTOR=%.2f for attempts.\n', initial_d2_scale_factor_to_apply, base_d2_growth_for_attempts);
        end
    end
    d2_iterative_base = d2_iterative_base * initial_d2_scale_factor_to_apply;
    d2_iterative_base = max(d2_iterative_base, 0.025); % Ensure it's not too excessively small, slightly reduced minimum


    dirs_to_try  = [sign(wrapToPi(thetaS_segment-thetaE_segment)), -sign(wrapToPi(thetaS_segment-thetaE_segment))];
    if dirs_to_try(1)==0, dirs_to_try(1)=1; end 
    if length(dirs_to_try) > 1 && dirs_to_try(2)==0, dirs_to_try(2)=-1; end 
    if length(dirs_to_try) > 1 && dirs_to_try(1) == dirs_to_try(2) 
        if dirs_to_try(1) == 1, dirs_to_try = [1, -1]; else, dirs_to_try = [-1, 1]; end
    end

    stats_input_segment = danger_metrics_original_segment; 
    score_input_segment = 0;
    if stats_input_segment.truck_collision > 0 || stats_input_segment.trailer_collision > 0
        score_input_segment = score_input_segment + COLLISION_PENALTY_ABSOLUTE * (stats_input_segment.truck_collision + stats_input_segment.trailer_collision);
    end
    score_input_segment = score_input_segment + (stats_input_segment.truck_close + stats_input_segment.trailer_close);
    
    bestOverallScore = score_input_segment; 
    x_best = x_curve_segment; 
    y_best = y_curve_segment;
    best_p1_overall = [NaN NaN]; 
    best_p2_overall = [NaN NaN];
    best_overall_stats = stats_input_segment;
    
    temp_per_side_diagnostic_handles = []; 
    fprintf('  Attempting Recker Mod (Seg. Global Idx %d, Arc: %.2fm, DeltaAng: %.2frad, Kappa: %.2f):\n', s_idx_global_start_of_segment, arc_segment, delta_angle_segment, kappa_segment);
    fprintf('    Initial segment score (from global eval): %.2f [TC:%d,TrC:%d,TkC:%d,TlC:%d]\n', score_input_segment, stats_input_segment.truck_collision, stats_input_segment.trailer_collision, stats_input_segment.truck_close, stats_input_segment.trailer_close);
    fprintf('    Using d1_fixed: %.2f, d2_iter_base_start (scaled): %.2f\n', d1_handle_fixed, d2_iterative_base);

    for dir_idx = 1:length(dirs_to_try)
        current_direction = dirs_to_try(dir_idx);
        fprintf('    Side (Direction %d):\n', current_direction);
        delete(temp_per_side_diagnostic_handles(isgraphics(temp_per_side_diagnostic_handles))); 
        temp_per_side_diagnostic_handles = [];
        
        effective_d2_growth_this_side = base_d2_growth_for_attempts;
        if arc_segment > LONG_CURVE_ARC_THRESHOLD
            effective_d2_growth_this_side = base_d2_growth_for_attempts * D2_GROWTH_ADJUSTMENT_FACTOR_LONG_CURVE;
            effective_d2_growth_this_side = max(1.01, effective_d2_growth_this_side); 
             if abs(effective_d2_growth_this_side - base_d2_growth_for_attempts) > 1e-3
                fprintf('        Info: Long curve (%.2fm). Base D2_GROWTH for this risk: %.2f. Adjusted for length to: %.3f\n', arc_segment, base_d2_growth_for_attempts, effective_d2_growth_this_side);
             end
        end
        
        % Corrected call to try_one_side (5 main arguments after current_direction)
        [x_try_side, y_try_side, score_side, p1_best_side, p2_best_side, handles_from_side_attempts, stats_best_side] = ... 
            try_one_side(current_direction, d1_handle_fixed, d2_iterative_base, arc_segment, effective_d2_growth_this_side); 
        
        temp_per_side_diagnostic_handles = handles_from_side_attempts; 
        if score_side < bestOverallScore
            bestOverallScore = score_side;
            x_best = x_try_side;
            y_best = y_try_side;
            best_p1_overall = p1_best_side;
            best_p2_overall = p2_best_side;
            best_overall_stats = stats_best_side;
            fprintf('      >> New Best Overall Score for this Curve: %.2f (from Side %d)\n', bestOverallScore, current_direction);
        end
    end
    
    delete(temp_per_side_diagnostic_handles(isgraphics(temp_per_side_diagnostic_handles))); 
    fprintf('  >> Chosen Final Score for Recker Modification = %.2f\n',bestOverallScore);
    
    modification_succeeded = (best_overall_stats.truck_collision == 0 && ...
                              best_overall_stats.trailer_collision == 0);
    
    final_diagnostic_handles = [];
    if ~all(isnan(best_p1_overall)) && ~all(isnan(best_p2_overall)) && (~isequal(x_best, x_curve_segment) || ~isequal(y_best, y_curve_segment)) 
        xS_local_final = x_curve_segment(1); yS_local_final = y_curve_segment(1);
        xE_local_final = x_curve_segment(end); yE_local_final = y_curve_segment(end);
        
        h_cp1 = plot(best_p1_overall(1),best_p1_overall(2),'go','MarkerFaceColor','g','MarkerSize',8, 'DisplayName', 'Chosen P1');
        h_cp2 = plot(best_p2_overall(1),best_p2_overall(2),'co','MarkerFaceColor','c','MarkerSize',8, 'DisplayName', 'Chosen P2');
        text(best_p1_overall(1), best_p1_overall(2), ' P1','Color','g','VerticalAlignment','bottom', 'FontWeight','bold');
        text(best_p2_overall(1), best_p2_overall(2), ' P2','Color','c','VerticalAlignment','bottom', 'FontWeight','bold');
        final_diagnostic_handles = [final_diagnostic_handles, h_cp1, h_cp2];
        
        h_line_S_P1 = plot([xS_local_final, best_p1_overall(1)], [yS_local_final, best_p1_overall(2)], 'g--','LineWidth',1.5);
        h_line_E_P2 = plot([xE_local_final, best_p2_overall(1)], [yE_local_final, best_p2_overall(2)], 'c--','LineWidth',1.5);
        set([h_line_S_P1, h_line_E_P2], 'HandleVisibility','off'); 
        final_diagnostic_handles = [final_diagnostic_handles, h_line_S_P1, h_line_E_P2];
    end

    if ~isempty(x_best)
        [truck_final_sim_for_plot, trailer_final_sim_for_plot] = simulate_vehicle_from_pose(x_best,y_best,...
            x_curve_segment(1),y_curve_segment(1),thetaS_segment,... 
            trailer_original_path.x(s_idx_global_start_of_segment), trailer_original_path.y(s_idx_global_start_of_segment), trailer_original_path.theta(s_idx_global_start_of_segment));

        if ~isempty(truck_final_sim_for_plot.x) && ~isempty(trailer_final_sim_for_plot.x)
             [status_final_plot, ~] = evaluate_trajectory_safety(truck_final_sim_for_plot,trailer_final_sim_for_plot,map,1,length(x_best)); 
            fprintf('    Final chosen path evaluation: TC:%d, TrC:%d, TkC:%d, TlC:%d. Success: %s\n', ...
                best_overall_stats.truck_collision, best_overall_stats.trailer_collision, ...
                best_overall_stats.truck_close, best_overall_stats.trailer_close, ...
                matlab.lang.makeValidName(string(modification_succeeded)));

            for ii_local = 1:length(status_final_plot) 
                face_alpha_val = 0.3; edge_color_val = 'k'; 
                plot_patch_final = false; patch_fc_final = 'none'; patch_ls_final = '-';
                if status_final_plot(ii_local).truck_flag == "collision"
                    patch_fc_final = 'red'; plot_patch_final = true; patch_ls_final = '-';
                elseif status_final_plot(ii_local).truck_flag == "close"
                    patch_fc_final = 'yellow'; plot_patch_final = true; patch_ls_final = '-';
                end
                if plot_patch_final && ii_local <= length(truck_final_sim_for_plot.x) 
                    trkPoly_f = get_rectangle(truck_final_sim_for_plot.x(ii_local),truck_final_sim_for_plot.y(ii_local),truck_final_sim_for_plot.theta(ii_local),g.Ltr,g.Wtr);
                    h_pf = patch(trkPoly_f(:,1), trkPoly_f(:,2), patch_fc_final, 'FaceAlpha',face_alpha_val,'EdgeColor',edge_color_val,'LineStyle',patch_ls_final,'HandleVisibility','off');
                    final_diagnostic_handles = [final_diagnostic_handles, h_pf];
                end
                
                plot_patch_final = false; 
                if status_final_plot(ii_local).trailer_flag == "collision"
                    patch_fc_final = 'red'; plot_patch_final = true; patch_ls_final = '--';
                elseif status_final_plot(ii_local).trailer_flag == "close"
                    patch_fc_final = 'yellow'; plot_patch_final = true; patch_ls_final = '--';
                end
                if plot_patch_final && ii_local <= length(trailer_final_sim_for_plot.x) 
                    trlPoly_f = get_rectangle(trailer_final_sim_for_plot.x(ii_local),trailer_final_sim_for_plot.y(ii_local),trailer_final_sim_for_plot.theta(ii_local),g.Ltl,g.Wtl);
                    h_pf = patch(trlPoly_f(:,1), trlPoly_f(:,2), patch_fc_final, 'FaceAlpha',face_alpha_val,'EdgeColor',edge_color_val,'LineStyle',patch_ls_final,'HandleVisibility','off');
                    final_diagnostic_handles = [final_diagnostic_handles, h_pf];
                end
            end
        else
            fprintf('    Warning: Simulation of final chosen path for plotting failed or produced empty output.\n');
        end
    elseif isempty(x_best) 
         modification_succeeded = false; 
    end
    drawnow; 

% try_one_side function definition
function [x_mod_best_for_side, y_mod_best_for_side, best_score_for_side, p1_best_this_side, p2_best_this_side, attempt_diagnostic_handles_group, stats_best_this_side] = ...
    try_one_side(k_direction, d1_fixed_param, d2_iter_start_val, arc_segment_passed, actual_d2_growth_factor) 
    
    d2_current_iter_val = d2_iter_start_val; 
    
    xS_local = x_curve_segment(1); yS_local = y_curve_segment(1);
    xE_local = x_curve_segment(end); yE_local = y_curve_segment(end);
    right_perp_vec = @(th)[ sin(th) -cos(th)];
    left_perp_vec  = @(th)[-sin(th)  cos(th)];
    
    best_score_for_side = inf;  
    x_mod_best_for_side = x_curve_segment; 
    y_mod_best_for_side = y_curve_segment;
    p1_best_this_side = [NaN NaN]; 
    p2_best_this_side = [NaN NaN];
    stats_best_this_side = struct('truck_collision',inf,'trailer_collision',inf,'truck_close',inf,'trailer_close',inf); 

    attempt_diagnostic_handles_group = []; 

    for attempt_num = 1:MAX_ATTEMPTS_PER_SIDE 
        d2_effective_for_calc = d2_current_iter_val; 
        max_permissible_d2 = max(d1_fixed_param * 1.5, arc_segment_passed * 0.6);
        d2_effective_for_calc = min(d2_effective_for_calc, max_permissible_d2);
        
        if k_direction > 0 
            perpS_vector = left_perp_vec(thetaS_segment); 
            perpE_vector = left_perp_vec(thetaE_segment);
        else 
            perpS_vector = right_perp_vec(thetaS_segment);
            perpE_vector = right_perp_vec(thetaE_segment); 
        end
        
        p1_coords_attempt = [xS_local yS_local] + d1_fixed_param*[cos(thetaS_segment) sin(thetaS_segment)] + d2_effective_for_calc*perpS_vector;
        p2_coords_attempt = [xE_local yE_local] - d1_fixed_param*[cos(thetaE_segment) sin(thetaE_segment)] + d2_effective_for_calc*perpE_vector;
        
        score_this_attempt = 0; 
        handles_crossed = segments_intersect([xS_local, yS_local], p1_coords_attempt, ...
                                             [xE_local, yE_local], p2_coords_attempt);
        if handles_crossed
            fprintf('        WARN: Control point handles S-P1 and E-P2 intersect for d1=%.2f, d2_eff=%.2f. Penalizing.\n', d1_fixed_param, d2_effective_for_calc);
            score_this_attempt = score_this_attempt + LOOP_PENALTY; 
        end

        num_bezier_points = max(30, length(x_curve_segment)); 
        t_parameter = linspace(0,1,num_bezier_points)'; 
        
        x_bezier_candidate = (1-t_parameter).^3*xS_local + ...
                             3*(1-t_parameter).^2.*t_parameter*p1_coords_attempt(1) + ...
                             3*(1-t_parameter).*t_parameter.^2*p2_coords_attempt(1) + ...
                             t_parameter.^3*xE_local;
        y_bezier_candidate = (1-t_parameter).^3*yS_local + ...
                             3*(1-t_parameter).^2.*t_parameter*p1_coords_attempt(2) + ...
                             3*(1-t_parameter).*t_parameter.^2*p2_coords_attempt(2) + ...
                             t_parameter.^3*yE_local;
        
        if length(x_bezier_candidate) < 2
            x_mod_candidate_path = x_curve_segment; 
            y_mod_candidate_path = y_curve_segment;
        else
            [x_mod_candidate_path,y_mod_candidate_path] = resample_bezier_to_N(x_bezier_candidate,y_bezier_candidate,length(x_curve_segment));
        end
        
        sim_init_idx = min(s_idx_global_start_of_segment, size(trailer_original_path.x,1));
        sim_init_idx = max(1, sim_init_idx); 

        initial_trailer_x_for_sim = trailer_original_path.x(sim_init_idx);
        initial_trailer_y_for_sim = trailer_original_path.y(sim_init_idx);
        initial_trailer_theta_for_sim = trailer_original_path.theta(sim_init_idx);
        
        [truck_sim_candidate,trailer_sim_candidate] = simulate_vehicle_from_pose(x_mod_candidate_path,y_mod_candidate_path,...
            xS_local,yS_local,thetaS_segment,... 
            initial_trailer_x_for_sim, initial_trailer_y_for_sim, initial_trailer_theta_for_sim);
        
        stats_candidate = struct('truck_collision',0,'trailer_collision',0,'truck_close',0,'trailer_close',0); 
        status_candidate = [];

        if isempty(x_mod_candidate_path) || isempty(truck_sim_candidate.x) || isempty(trailer_sim_candidate.x)
            stats_candidate.truck_collision = 100; stats_candidate.trailer_collision = 100; 
            stats_candidate.truck_close = 100; stats_candidate.trailer_close = 100;
            score_this_attempt = score_this_attempt + COLLISION_PENALTY_ABSOLUTE * 200; 
        else
            [status_candidate,current_stats_candidate] = evaluate_trajectory_safety(truck_sim_candidate,trailer_sim_candidate,map,1,length(x_mod_candidate_path));
            stats_candidate = current_stats_candidate; 
            
            if stats_candidate.truck_collision > 0 || stats_candidate.trailer_collision > 0
                score_this_attempt = score_this_attempt + COLLISION_PENALTY_ABSOLUTE * (stats_candidate.truck_collision + stats_candidate.trailer_collision);
            end
            score_this_attempt = score_this_attempt + (stats_candidate.truck_close+stats_candidate.trailer_close); 
            
            if ~isempty(x_bezier_candidate) && getOccupancy(map,midpoint_of_curve(x_bezier_candidate,y_bezier_candidate)) == 1 
                score_this_attempt = score_this_attempt + OUTER_WALL_HIT_PENALTY;
            end
        end
        
        is_problematic_for_logging = stats_candidate.truck_collision > 0 || stats_candidate.trailer_collision > 0 || ...
                                     stats_candidate.truck_close > 0 || stats_candidate.trailer_close > 0 || ...
                                     handles_crossed; 
        
        log_this_attempt = false;
        if attempt_num == 1 || attempt_num == MAX_ATTEMPTS_PER_SIDE || is_problematic_for_logging
            log_this_attempt = true;
        elseif score_this_attempt < best_score_for_side 
             log_this_attempt = true;
        end

        if log_this_attempt
            fprintf('        Att %2d (d1:%.2f, d2i:%.2f, d2e:%.2f): Scr %.2f (LoopP:%d) [TC:%d,TrC:%d,TkC:%d,TlC:%d]\n',...
                attempt_num, d1_fixed_param, d2_current_iter_val, d2_effective_for_calc, score_this_attempt, handles_crossed*LOOP_PENALTY, ...
                stats_candidate.truck_collision, stats_candidate.trailer_collision, ...
                stats_candidate.truck_close, stats_candidate.trailer_close);
        end
        
        temp_patches_for_this_attempt = []; 
        if ~isempty(status_candidate) && is_problematic_for_logging 
             h_cand_path = plot(x_mod_candidate_path, y_mod_candidate_path, ':', 'Color', [0.85, 0.85, 0.85], 'LineWidth',0.5,'HandleVisibility','off');
             temp_patches_for_this_attempt = [temp_patches_for_this_attempt, h_cand_path];
            for ii_local = 1:length(status_candidate) 
                face_alpha_val = 0.2; edge_color_val = 'k'; 
                plot_this_patch_attempt = false; patch_fc_attempt = 'none'; patch_ls_attempt = '-';
                if status_candidate(ii_local).truck_flag == "collision"
                    patch_fc_attempt = 'red'; plot_this_patch_attempt = true; patch_ls_attempt = '-';
                elseif status_candidate(ii_local).truck_flag == "close"
                    patch_fc_attempt = 'yellow'; plot_this_patch_attempt = true; patch_ls_attempt = '-';
                end
                if plot_this_patch_attempt
                    trkPoly_att = get_rectangle(truck_sim_candidate.x(ii_local),truck_sim_candidate.y(ii_local),truck_sim_candidate.theta(ii_local),g.Ltr,g.Wtr);
                    h_p_att = patch(trkPoly_att(:,1), trkPoly_att(:,2), patch_fc_attempt, 'FaceAlpha',face_alpha_val,'EdgeColor',edge_color_val,'LineStyle',patch_ls_attempt,'HandleVisibility','off');
                    temp_patches_for_this_attempt = [temp_patches_for_this_attempt, h_p_att];
                end
                
                plot_this_patch_attempt = false; 
                if status_candidate(ii_local).trailer_flag == "collision"
                    patch_fc_attempt = 'red'; plot_this_patch_attempt = true; patch_ls_attempt = '--';
                elseif status_candidate(ii_local).trailer_flag == "close"
                    patch_fc_attempt = 'yellow'; plot_this_patch_attempt = true; patch_ls_attempt = '--';
                end
                if plot_this_patch_attempt
                    trlPoly_att = get_rectangle(trailer_sim_candidate.x(ii_local),trailer_sim_candidate.y(ii_local),trailer_sim_candidate.theta(ii_local),g.Ltl,g.Wtl);
                    h_p_att = patch(trlPoly_att(:,1), trlPoly_att(:,2), patch_fc_attempt, 'FaceAlpha',face_alpha_val,'EdgeColor',edge_color_val,'LineStyle',patch_ls_attempt,'HandleVisibility','off');
                    temp_patches_for_this_attempt = [temp_patches_for_this_attempt, h_p_att];
                end
            end
        end
        attempt_diagnostic_handles_group = [attempt_diagnostic_handles_group, temp_patches_for_this_attempt]; 
        if ~isempty(temp_patches_for_this_attempt)
            drawnow limitrate; 
        end
        
        if score_this_attempt < best_score_for_side
            if log_this_attempt && (attempt_num > 1 || (attempt_num == 1 && ~is_problematic_for_logging)) 
                 fprintf('          >> New Best Score for this Side: %.2f\n', score_this_attempt);
            end
            best_score_for_side = score_this_attempt; 
            x_mod_best_for_side = x_mod_candidate_path; 
            y_mod_best_for_side = y_mod_candidate_path;
            p1_best_this_side = p1_coords_attempt; 
            p2_best_this_side = p2_coords_attempt;
            stats_best_this_side = stats_candidate;
            % min_clear_best_this_side_info = min_clear_val_this_attempt_info; % Removed
            
            if stats_candidate.truck_collision == 0 && stats_candidate.trailer_collision == 0 && ~handles_crossed
                fprintf('        Collision-free solution found. Score: %.2f. Breaking attempts.\n', score_this_attempt);
                break; 
            end
        end
        
        d2_current_iter_val = d2_current_iter_val * actual_d2_growth_factor; 
        
    end 
    
end 
end 
%%
function v_pad = pad_curve(v, pad)
    N = length(v);
    v_pad = [repmat(v(1), pad, 1); v; repmat(v(end), pad, 1)];
    if length(v_pad) > N + 2*pad
        v_pad = v_pad(1:N + 2*pad);  % clip
    end
end
function idx = find_closest_idx(x, y, x_list, y_list)
    if isempty(x_list) || isempty(y_list)
        idx = 1; 
        warning('find_closest_idx:empty_input', 'Input list_x or list_y is empty.');
        return;
    end
    dists = hypot(x_list - x, y_list - y);
    [~, idx] = min(dists);
end
function [p1, p2] = control_points(xS, yS, xE, yE, thetaS, thetaE, d1, d2, k)
    right_perp = @(theta)[sin(theta), -cos(theta)];
    left_perp  = @(theta)[-sin(theta), cos(theta)];
    if k > 0
        perpS = left_perp(thetaS); perpE = left_perp(thetaE);
    else
        perpS = right_perp(thetaS); perpE = right_perp(thetaE);
    end
    p1 = [xS, yS] + d1 * [cos(thetaS), sin(thetaS)] + d2 * perpS;
    p2 = [xE, yE] - d1 * [cos(thetaE), sin(thetaE)] + d2 * perpE;

    plot([xS p1(1)], [yS p1(2)], 'g--', 'LineWidth', 1.2); 
    plot([xE p2(1)], [yE p2(2)], 'c--', 'LineWidth', 1.2);  
    plot(p1(1), p1(2), 'go', 'MarkerSize', 8, 'LineWidth', 1.5);
    plot(p2(1), p2(2), 'co', 'MarkerSize', 8, 'LineWidth', 1.5);
    text(p1(1), p1(2), 'p1', 'Color', 'g', 'FontSize', 8, 'FontWeight', 'bold');
    text(p2(1), p2(2), 'p2', 'Color', 'c', 'FontSize', 8, 'FontWeight', 'bold');
end
function crossed = is_crossing(xS, yS, p1, xE, yE, p2) 
    crossed = segments_intersect([xS, yS], p1, [xE, yE], p2);
end
function tf = segments_intersect(p1_seg1, p2_seg1, p1_seg2, p2_seg2)
    o1 = orientation(p1_seg1, p2_seg1, p1_seg2); 
    o2 = orientation(p1_seg1, p2_seg1, p2_seg2); 
    o3 = orientation(p1_seg2, p2_seg2, p1_seg1); 
    o4 = orientation(p1_seg2, p2_seg2, p2_seg1); 
    
    if o1 ~= 0 && o2 ~= 0 && o3 ~= 0 && o4 ~= 0
        if o1 ~= o2 && o3 ~= o4
            tf = true;
            return;
        end
    end
    
    if o1 == 0 && on_segment(p1_seg1, p1_seg2, p2_seg1), tf = true; return; end
    if o2 == 0 && on_segment(p1_seg1, p2_seg2, p2_seg1), tf = true; return; end
    if o3 == 0 && on_segment(p1_seg2, p1_seg1, p2_seg2), tf = true; return; end
    if o4 == 0 && on_segment(p1_seg2, p2_seg1, p2_seg2), tf = true; return; end
    
    tf = false; 
end

function o = orientation(p, q, r)
    val = (q(2) - p(2)) * (r(1) - q(1)) - ...
          (q(1) - p(1)) * (r(2) - q(2));
    if abs(val) < 1e-9 
        o = 0;  
    elseif val > 0
        o = 1;  
    else
        o = 2;  
    end
end

function tf = on_segment(p, q, r)
    tf = (q(1) <= max(p(1), r(1)) && q(1) >= min(p(1), r(1)) && ...
          q(2) <= max(p(2), r(2)) && q(2) >= min(p(2), r(2)));
end

function pts = interpolate_line(p1, p2, n)
    if n < 1, pts = []; return; end 
    if n == 1, pts = p1; return; end 
    t = linspace(0,1,n)';
    pts = p1 + t .* (p2 - p1);
end
function [x_out, y_out] = resample_bezier_to_N(x_bez, y_bez, N)
    if isempty(x_bez) || length(x_bez) < 2
        x_out = x_bez; y_out = y_bez; return;
    end
    if N <= 0 
       x_out = []; y_out = [];
       if N == 1 && ~isempty(x_bez), x_out = x_bez(1); y_out = y_bez(1); end 
       return;
    end
    dx = diff(x_bez);
    dy = diff(y_bez);
    arc_lengths = [0; cumsum(sqrt(dx.^2 + dy.^2))];
    total_length = arc_lengths(end);
    if total_length < 1e-6
        x_out = repmat(x_bez(1), N, 1);
        y_out = repmat(y_bez(1), N, 1);
        return;
    end
    s_target = linspace(0, total_length, N);
    x_out = interp1(arc_lengths, x_bez, s_target, 'linear')';
    y_out = interp1(arc_lengths, y_bez, s_target, 'linear')';
    x_out = x_out(:); y_out = y_out(:);
end
function rect = get_rectangle(x, y, theta, L, W)
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    corners = [-L/2 -W/2; -L/2 W/2; L/2 W/2; L/2 -W/2]; 
    rect = (R * corners')' + repmat([x y], 4, 1); 
end
%% ========== Pure‑pursuit simulator from arbitrary pose ==========
function [truck,trailer] = simulate_vehicle_from_pose(x_ref,y_ref,...
        x0,y0,theta0,tr_x0,tr_y0,tr_theta0)
g = geom(); 
n = length(x_ref);
if n == 0 
    truck = struct('x',[],'y',[],'theta',[]);
    trailer = struct('x',[],'y',[],'theta',[]);
    return;
end
truck   = struct('x',zeros(n,1),'y',zeros(n,1),'theta',zeros(n,1));
trailer = struct('x',zeros(n,1),'y',zeros(n,1),'theta',zeros(n,1));
truck.x(1)=x0; truck.y(1)=y0; truck.theta(1)=theta0;
trailer.x(1)=tr_x0; trailer.y(1)=tr_y0; trailer.theta(1)=tr_theta0;

if n < 2 
    return;
end

for i = 1:n-1
    xf = truck.x(i) + g.Ltr/2*cos(truck.theta(i)); 
    yf = truck.y(i) + g.Ltr/2*sin(truck.theta(i));
    
    dist_sq_to_path = (x_ref-xf).^2 + (y_ref-yf).^2; 
    [~,idx_closest_on_ref] = min(dist_sq_to_path);
    
    current_search_idx = idx_closest_on_ref;
    target_idx_candidate = idx_closest_on_ref;

    while current_search_idx <= n
        dist_to_current_search_pt_sq = (x_ref(current_search_idx)-xf)^2 + (y_ref(current_search_idx)-yf)^2;
        if dist_to_current_search_pt_sq >= g.look^2
            target_idx_candidate = current_search_idx;
            break; 
        end
        if current_search_idx == n 
            target_idx_candidate = n;
            break;
        end
        current_search_idx = current_search_idx + 1;
    end
    idx_la = target_idx_candidate;
    
    xt = x_ref(idx_la); yt = y_ref(idx_la);
    alpha = atan2(yt-truck.y(i),xt-truck.x(i)) - truck.theta(i); 
    alpha = wrapToPi(alpha);

    L_eff = sqrt((xt-truck.x(i))^2 + (yt-truck.y(i))^2);
    L_eff = max(L_eff, 0.1); 

    delta = atan2(2*g.Ltr*sin(alpha),L_eff); 
    MAX_STEER = deg2rad(40); 
    delta = max(-MAX_STEER, min(MAX_STEER, delta));

    truck.x(i+1)   = truck.x(i)   + g.v*cos(truck.theta(i))*g.dt;
    truck.y(i+1)   = truck.y(i)   + g.v*sin(truck.theta(i))*g.dt;
    truck.theta(i+1)= wrapToPi(truck.theta(i) + (g.v*tan(delta)/g.Ltr)*g.dt);

    hx = truck.x(i+1)-g.Ltr/2*cos(truck.theta(i+1)); 
    hy = truck.y(i+1)-g.Ltr/2*sin(truck.theta(i+1));
    trailer.theta(i+1) = wrapToPi(atan2(hy-trailer.y(i),hx-trailer.x(i))); 
    trailer.x(i+1) = hx - (g.Lh+g.Ltl/2)*cos(trailer.theta(i+1)); 
    trailer.y(i+1) = hy - (g.Lh+g.Ltl/2)*sin(trailer.theta(i+1));
end
end
%% ========== Global geometry & controller constants ==========
function g = geom()
    g.Ltr  = 2.20;   
    g.Wtr  = 1.00;   
    g.Ltl  = 1.50;   
    g.Wtl  = 1.00;   
    g.Lh   = 2.00;   
    g.dt   = 0.05;   
    g.v    = 0.50;   
    g.look = 1.00;   
end

function grad = gradient_to_nearest_obstacle(pt,map)
    eps  = 0.05; 
    dirs = [ 1  0 ; -1  0 ; 0  1 ; 0 -1 ] * eps; 
    occ_vals  = zeros(4,1);
    for k_probe = 1:4
       occ_vals(k_probe) = getOccupancy(map, pt + dirs(k_probe,:));
    end
    grad_x = occ_vals(1) - occ_vals(2); 
    grad_y = occ_vals(3) - occ_vals(4); 
    grad = [grad_x, grad_y];
    
    if norm(grad) > 1e-6, grad = grad/norm(grad); end 
end
%% ========== Small utilities used above ==========
function pt = midpoint_of_curve(xb,yb)
    if isempty(xb) || isempty(yb)
        pt = [NaN NaN]; return;
    end
    mid_idx = round(numel(xb)/2);
    if mid_idx < 1, mid_idx = 1; end
    if mid_idx > numel(xb), mid_idx = numel(xb); end
    pt = [ xb(mid_idx) , yb(mid_idx) ];
end
function [d_min,idx_global] = min_clearance(obj,map,s_local,e_local)
    % This function remains for potential future diagnostic use, 
    % but its output is NOT used for scoring or success criteria.
    d_min = inf; 
    idx_global = NaN; 
    OCCUPIED_VALUE = 1; 

    if isempty(obj.x) || s_local > e_local || s_local < 1 || e_local > length(obj.x)
        return;
    end
    
    occ_matrix = occupancyMatrix(map); 
    res = map.Resolution;                    
    DT_pixels  = bwdist(occ_matrix == OCCUPIED_VALUE); 
    DT_meters  = DT_pixels/res; 
    
    xs_world_segment = obj.x(s_local:e_local);
    ys_world_segment = obj.y(s_local:e_local);

    grid_xs  = round( (xs_world_segment - map.XWorldLimits(1))*res ) + 1;
    grid_ys  = round( (ys_world_segment - map.YWorldLimits(1))*res ) + 1; 
    
    grid_xs  = min(max(grid_xs,1),size(DT_meters,2)); 
    grid_ys  = min(max(grid_ys,1),size(DT_meters,1)); 
    
    linear_indices = sub2ind(size(DT_meters),grid_ys,grid_xs);
    
    clearances_at_points = DT_meters(linear_indices);
    if isempty(clearances_at_points)
        return;
    end

    [d_min_segment,rel_idx_in_segment] = min(clearances_at_points);
    d_min = d_min_segment;
    idx_global = s_local + rel_idx_in_segment - 1; 
end
function flag = flag_from_check(collision, near)
    if collision
        flag = "collision";
    elseif near
        flag = "close";
    else
        flag = "safe";
    end
end

