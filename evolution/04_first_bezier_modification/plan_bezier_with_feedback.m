function [x_smooth, y_smooth] = plan_bezier_with_feedback(x_ref, y_ref, theta_ref, ...
                                                          curve_bounds_padded, ...
                                                          map, d1, d2_init, d2_min, d2_max, pivot_thresh)
    max_attempts = 20;
    d2 = d2_init;

    for attempt = 1:max_attempts
        fprintf('Attempt %d — trying d2 = %.3f\n', attempt, d2);

        % 1. Apply Bézier with current d2
        [x_mod, y_mod] = modify_curves_bezier(x_ref, y_ref, theta_ref, ...
                                              curve_bounds_padded, d1, d2, map);

        % 2. Simulate vehicle + trailer
        [truck, trailer] = simulate_vehicle(x_mod, y_mod);
        pivot_angle = wrapToPi(truck.theta - trailer.theta);
        max_pivot = max(abs(rad2deg(pivot_angle)));

        % 3. Check for trailer collisions
        trailer_path = [trailer.x, trailer.y];
        collision = false;
        for i = 1:size(trailer_path,1)
            if getOccupancy(map, trailer_path(i,:)) > 0.1
                collision = true;
                break;
            end
        end

        % 4. Decision logic
        if collision
            fprintf("🚧 Collision detected — INCREASING d2...\n");
            d2 = min(d2 * 1.3, d2_max);
        elseif max_pivot > pivot_thresh
            fprintf("⚠️ Pivot angle %.1f° too sharp — REDUCING d2...\n", max_pivot);
            d2 = max(d2 * 0.8, d2_min);
        else
            fprintf("✅ Path accepted — max pivot %.1f°\n", max_pivot);
            break;
        end
    end

    % 5. Smooth final result
    x_smooth = smoothdata(x_mod, 'gaussian', 10);
    y_smooth = smoothdata(y_mod, 'gaussian', 10);
end