function [x_final, y_final] = plan_bezier_dual_d2(x_ref, y_ref, theta_ref, ...
                                                  curve_bounds, map, ...
                                                  d1, d2_base, pivot_thresh)

    % Versioni d2
    d2_low = d2_base * 0.7;
    d2_high = d2_base * 1.3;

    % === Versione 1: curva stretta ===
    [x_lo, y_lo] = modify_curves_bezier(x_ref, y_ref, theta_ref, curve_bounds, d1, d2_low, map);
    [truck_lo, trailer_lo] = simulate_vehicle(x_lo, y_lo);
    pivot_lo = max(abs(rad2deg(wrapToPi(truck_lo.theta - trailer_lo.theta))));
    coll_lo = detect_trailer_collision(map, trailer_lo);

    % === Versione 2: curva larga ===
    [x_hi, y_hi] = modify_curves_bezier(x_ref, y_ref, theta_ref, curve_bounds, d1, d2_high, map);
    [truck_hi, trailer_hi] = simulate_vehicle(x_hi, y_hi);
    pivot_hi = max(abs(rad2deg(wrapToPi(truck_hi.theta - trailer_hi.theta))));
    coll_hi = detect_trailer_collision(map, trailer_hi);

    % === Logica di selezione ===
    figure('Name','Dual d2 Evaluation'); clf; hold on;
    show(map); title('Collision / Pivot Evaluation'); axis equal;

    if ~coll_lo && ~coll_hi
        % Nessuna collisione → scegli quello col pivot minore
        if pivot_lo <= pivot_hi
            x_final = smoothdata(x_lo, 'gaussian', 10);
            y_final = smoothdata(y_lo, 'gaussian', 10);
            text(truck_lo.x(1), truck_lo.y(1), 'Chosen: LOW d2 (pivot)', 'Color', 'b');
        else
            x_final = smoothdata(x_hi, 'gaussian', 10);
            y_final = smoothdata(y_hi, 'gaussian', 10);
            text(truck_hi.x(1), truck_hi.y(1), 'Chosen: HIGH d2 (pivot)', 'Color', 'g');
        end
    elseif ~coll_lo
        x_final = smoothdata(x_lo, 'gaussian', 10);
        y_final = smoothdata(y_lo, 'gaussian', 10);
        text(truck_lo.x(1), truck_lo.y(1), 'Chosen: LOW d2 (high d2 collision)', 'Color', 'r');
    elseif ~coll_hi
        x_final = smoothdata(x_hi, 'gaussian', 10);
        y_final = smoothdata(y_hi, 'gaussian', 10);
        text(truck_hi.x(1), truck_hi.y(1), 'Chosen: HIGH d2 (low d2 collision)', 'Color', 'm');
    else
        warning('🚨 Both paths collide — fallback to base d2');
        [x_base, y_base] = modify_curves_bezier(x_ref, y_ref, theta_ref, curve_bounds, d1, d2_base, map);
        x_final = smoothdata(x_base, 'gaussian', 10);
        y_final = smoothdata(y_base, 'gaussian', 10);
        text(x_final(1), y_final(1), 'Fallback: base d2', 'Color', 'k');
    end
end

function collided = detect_trailer_collision(map, trailer)
    collided = false;
    for i = 1:length(trailer.x)
        if getOccupancy(map, [trailer.x(i), trailer.y(i)]) > 0.1
            plot(trailer.x(i), trailer.y(i), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
            text(trailer.x(i), trailer.y(i)+0.5, 'Collision!', 'Color', 'r', 'FontWeight', 'bold');
            collided = true;
            return;
        end
    end
end
