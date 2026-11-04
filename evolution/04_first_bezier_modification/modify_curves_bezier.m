% % function [x_out, y_out] = modify_curves_bezier(x_ref, y_ref, theta_ref, curve_bounds, d1, d2)
% %     x_ref = x_ref(:); y_ref = y_ref(:); 
% %     theta_ref = unwrap(theta_ref(:));
% %     x_out = []; y_out = [];
% %     n = 100; last_idx = 1;
% %     prev_p2 = []; prev_k = 0;
% % 
% %     % Physical constants
% %     MIN_CONTROL_DIST = 1.8; 
% %     MAX_REACH_FACTOR = 0.5; 
% %     BLEND_WEIGHT = 0.4; 
% % 
% %     % Corrected perpendicular directions
% %     right_perp = @(theta) [sin(theta), -cos(theta)];  % 90° clockwise
% %     left_perp = @(theta) [-sin(theta), cos(theta)];   % 90° counter-clockwise
% % 
% %     for c = 1:size(curve_bounds,1)
% %         s_idx = curve_bounds(c,1); e_idx = curve_bounds(c,2);
% %         xS = x_ref(s_idx); yS = y_ref(s_idx);
% %         xE = x_ref(e_idx); yE = y_ref(e_idx);
% %         thetaS = theta_ref(s_idx); thetaE = theta_ref(e_idx);
% %         seg_vec = [xE-xS, yE-yS]; seg_len = norm(seg_vec);
% % 
% %         % 1. Stable Curvature Detection --------------------------------
% %         mid_idx = round(mean([s_idx, e_idx]));
% %         P_mid = [x_ref(mid_idx), y_ref(mid_idx)];
% %         segment_dir = [cos(thetaS), sin(thetaS)];
% %         vec_to_mid = P_mid - [xS, yS];
% %         cross_prod = segment_dir(1)*vec_to_mid(2) - segment_dir(2)*vec_to_mid(1);
% %         k = sign(cross_prod); % Positive = left turn, Negative = right turn
% % 
% %         % 2. Control Point Generation ----------------------------------
% %         d1_eff = max(d1*seg_len, MIN_CONTROL_DIST);
% %         d2_eff = max(d2*seg_len, MIN_CONTROL_DIST);
% % 
% %         if k > 0 % Left turn: control points on RIGHT side
% %             p1_raw = [xS, yS] + d1_eff*[cos(thetaS), sin(thetaS)] + d2_eff*right_perp(thetaS);
% %             p2_raw = [xE, yE] - d1_eff*[cos(thetaE), sin(thetaE)] + d2_eff*right_perp(thetaE);
% %         else % Right turn: control points on LEFT side
% %             p1_raw = [xS, yS] + d1_eff*[cos(thetaS), sin(thetaS)] + d2_eff*left_perp(thetaS);
% %             p2_raw = [xE, yE] - d1_eff*[cos(thetaE), sin(thetaE)] + d2_eff*left_perp(thetaE);
% %         end
% % 
% %         % 3. Transition Handling ---------------------------------------
% %         if ~isempty(prev_p2) && (k == prev_k)
% %             p1 = BLEND_WEIGHT*prev_p2 + (1-BLEND_WEIGHT)*p1_raw;
% %         else
% %             p1 = p1_raw;
% %         end
% % 
% %         % 4. Clamping -------------------------------------------------
% %         max_reach = seg_len * MAX_REACH_FACTOR;
% %         p1 = clamp_control_point([xS, yS], p1, max_reach);
% %         p2 = clamp_control_point([xE, yE], p2_raw, max_reach);
% % 
% %         % 5. Bézier Generation ----------------------------------------
% %         t = linspace(0,1,n)';
% %         bez_x = (1-t).^3*xS + 3*(1-t).^2.*t*p1(1) + 3*(1-t).*t.^2*p2(1) + t.^3*xE;
% %         bez_y = (1-t).^3*yS + 3*(1-t).^2.*t*p1(2) + 3*(1-t).*t.^2*p2(2) + t.^3*yE;
% % 
% %         % 6. Debug Plotting -------------------------------------------
% %         figure(1); hold on;
% %         plot([xS, xE], [yS, yE], 'k:', 'LineWidth', 0.5);
% %         plot(p1(1), p1(2), 'bd', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
% %         plot(p2(1), p2(2), 'ms', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
% %         text(mean([xS, xE]), mean([yS, yE]), sprintf('k=%d', k), 'HorizontalAlignment', 'center');
% %         plot(bez_x, bez_y, 'LineWidth', 2.5, 'Color', [0 0.5 0]);
% % 
% %         % 7. Append Results -------------------------------------------
% %         if last_idx < s_idx
% %             x_out = [x_out; x_ref(last_idx:s_idx-1)];
% %             y_out = [y_out; y_ref(last_idx:s_idx-1)];
% %         end
% %         x_out = [x_out; bez_x]; y_out = [y_out; bez_y];
% %         last_idx = e_idx + 1;
% %         prev_p2 = p2; prev_k = k;
% %     end
% % 
% %     if last_idx <= length(x_ref)
% %         x_out = [x_out; x_ref(last_idx:end)];
% %         y_out = [y_out; y_ref(last_idx:end)];
% %     end
% % end
% % 
% % function p_clamped = clamp_control_point(origin, target, max_dist)
% %     vec = target - origin;
% %     dist = norm(vec);
% %     if dist > max_dist
% %         p_clamped = origin + vec*(max_dist/dist);
% %     else
% %         p_clamped = target;
% %     end
% % end
% % 
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 
% % % function [x_out, y_out] = modify_curves_bezier(x_ref, y_ref, theta_ref, curve_bounds, d1, d2)
% % %     x_ref = x_ref(:); y_ref = y_ref(:); 
% % %     theta_ref = unwrap(theta_ref(:));
% % %     x_out = []; y_out = [];
% % %     n = 100; % Points per Bézier segment
% % %     last_idx = 1;
% % % 
% % %     for c = 1:size(curve_bounds,1)
% % %         s_idx = curve_bounds(c,1); 
% % %         e_idx = curve_bounds(c,2);
% % % 
% % %         % Extract segment data
% % %         xS = x_ref(s_idx); yS = y_ref(s_idx);
% % %         xE = x_ref(e_idx); yE = y_ref(e_idx);
% % %         thetaS = theta_ref(s_idx); 
% % %         thetaE = theta_ref(e_idx);
% % %         seg_len = norm([xE-xS, yE-yS]);
% % % 
% % %         % 1. Curvature detection via midpoint cross product
% % %         mid_idx = round(mean([s_idx, e_idx]));
% % %         P_mid = [x_ref(mid_idx), y_ref(mid_idx)];
% % %         segment_dir = [cos(thetaS), sin(thetaS)];
% % %         vec_to_mid = P_mid - [xS, yS];
% % %         cross_prod = segment_dir(1)*vec_to_mid(2) - segment_dir(2)*vec_to_mid(1);
% % %         k = sign(cross_prod); % +1=left, -1=right
% % % 
% % %         % 2. Perpendicular directions
% % %         right_perp = @(theta) [sin(theta), -cos(theta)]; 
% % %         left_perp = @(theta) [-sin(theta), cos(theta)]; 
% % % 
% % %         if k > 0 % Left turn: control points on RIGHT
% % %             perpS = right_perp(thetaS);
% % %             perpE = right_perp(thetaE);
% % %         else % Right turn: control points on LEFT
% % %             perpS = left_perp(thetaS);
% % %             perpE = left_perp(thetaE);
% % %         end
% % % 
% % %         % 3. Control point generation
% % %         d1_eff = max(d1*seg_len, 1.5); % Minimum distance
% % %         d2_eff = max(d2*seg_len, 1.5);
% % % 
% % %         p1 = [xS, yS] + d1_eff*[cos(thetaS), sin(thetaS)] + d2_eff*perpS;
% % %         p2 = [xE, yE] - d1_eff*[cos(thetaE), sin(thetaE)] + d2_eff*perpE;
% % % 
% % %         % 4. Clamp control points to segment length
% % %         max_reach = seg_len * 0.35;
% % %         p1 = clamp_control_point([xS, yS], p1, max_reach);
% % %         p2 = clamp_control_point([xE, yE], p2, max_reach);
% % % 
% % %         % 5. Bézier generation
% % %         t = linspace(0,1,n)';
% % %         bez_x = (1-t).^3*xS + 3*(1-t).^2.*t*p1(1) + 3*(1-t).*t.^2*p2(1) + t.^3*xE;
% % %         bez_y = (1-t).^3*yS + 3*(1-t).^2.*t*p1(2) + 3*(1-t).*t.^2*p2(2) + t.^3*yE;
% % % 
% % %         % 6. Append results
% % %         if last_idx < s_idx
% % %             x_out = [x_out; x_ref(last_idx:s_idx-1)];
% % %             y_out = [y_out; y_ref(last_idx:s_idx-1)];
% % %         end
% % %         x_out = [x_out; bez_x]; 
% % %         y_out = [y_out; bez_y];
% % %         last_idx = e_idx + 1;
% % %     end
% % % 
% % %     % Append remaining points
% % %     if last_idx <= length(x_ref)
% % %         x_out = [x_out; x_ref(last_idx:end)];
% % %         y_out = [y_out; y_ref(last_idx:end)];
% % %     end
% % % end
% % % 
% % % function p_clamped = clamp_control_point(origin, target, max_dist)
% % %     vec = target - origin;
% % %     dist = norm(vec);
% % %     if dist > max_dist
% % %         p_clamped = origin + vec*(max_dist/dist);
% % %     else
% % %         p_clamped = target;
% % %     end
% % % end
% 
% function [x_out, y_out] = modify_curves_bezier(x_ref, y_ref, theta_ref, curve_bounds, d1, d2)
%     x_ref = x_ref(:); y_ref = y_ref(:); 
%     theta_ref = unwrap(theta_ref(:));
%     x_out = []; y_out = [];
%     n = 100; last_idx = 1;
% 
%     % Parameters for outward enhancement
%     OUTWARD_FACTOR = 2;  % Increase to push curves outward more
%     MAX_REACH = 0.70;      % 45% of segment length
% 
%     figure(1); clf; hold on; % Initialize debug plot
%     plot(x_ref, y_ref, 'k--', 'DisplayName', 'Original Path');
% 
%     for c = 1:size(curve_bounds,1)
%         s_idx = curve_bounds(c,1); e_idx = curve_bounds(c,2);
%         xS = x_ref(s_idx); yS = y_ref(s_idx);
%         xE = x_ref(e_idx); yE = y_ref(e_idx);
%         thetaS = theta_ref(s_idx); thetaE = theta_ref(e_idx);
%         seg_len = norm([xE-xS, yE-yS]);
% 
%         % 1. Curvature detection (cross product method)
%         mid_idx = round(mean([s_idx, e_idx]));
%         P_mid = [x_ref(mid_idx), y_ref(mid_idx)];
%         segment_dir = [cos(thetaS), sin(thetaS)];
%         vec_to_mid = P_mid - [xS, yS];
%         cross_val = segment_dir(1)*vec_to_mid(2) - segment_dir(2)*vec_to_mid(1);
%         k = sign(cross_val); % +1=left turn, -1=right turn
% 
%         % 2. Perpendicular directions (visually verified)
%         right_perp = @(theta) [sin(theta), -cos(theta)] * OUTWARD_FACTOR;
%         left_perp = @(theta) [-sin(theta), cos(theta)] * OUTWARD_FACTOR;
% 
%         if k > 0 % Left turn: control points on RIGHT side
%             perpS = right_perp(thetaS);
%             perpE = right_perp(thetaE);
%             curve_color = [0 0.7 0]; % Dark green for left turns
%         else % Right turn: control points on LEFT side
%             perpS = left_perp(thetaS);
%             perpE = left_perp(thetaE);
%             curve_color = [0.7 0 0]; % Dark red for right turns
%         end
% 
%         % 3. Control point generation
%         d1_eff = max(d1*seg_len, 1.5);
%         d2_eff = max(d2*seg_len, 1.5);
% 
%         p1 = [xS, yS] + d1_eff*[cos(thetaS), sin(thetaS)] + d2_eff*perpS;
%         p2 = [xE, yE] - d1_eff*[cos(thetaE), sin(thetaE)] + d2_eff*perpE;
% 
%         % 4. Clamping
%         p1 = clamp_control_point([xS, yS], p1, seg_len*MAX_REACH);
%         p2 = clamp_control_point([xE, yE], p2, seg_len*MAX_REACH);
% 
%         % 5. Bézier curve
%         t = linspace(0,1,n)';
%         bez_x = (1-t).^3*xS + 3*(1-t).^2.*t*p1(1) + 3*(1-t).*t.^2*p2(1) + t.^3*xE;
%         bez_y = (1-t).^3*yS + 3*(1-t).^2.*t*p1(2) + 3*(1-t).*t.^2*p2(2) + t.^3*yE;
% 
%         % 6. Debug plotting
%         plot(xS, yS, 'go', 'MarkerSize', 8, 'LineWidth', 2); % Start marker
%         plot(xE, yE, 'ro', 'MarkerSize', 8, 'LineWidth', 2); % End marker
%         plot(p1(1), p1(2), 'b*', 'MarkerSize', 12); % Control point 1
%         plot(p2(1), p2(2), 'm*', 'MarkerSize', 12); % Control point 2
%         plot([xS, p1(1)], [yS, p1(2)], 'b--'); % Control line 1
%         plot([xE, p2(1)], [yE, p2(2)], 'm--'); % Control line 2
%         plot(bez_x, bez_y, 'LineWidth', 2, 'Color', curve_color); % Curve itself
%         text(mean([xS,xE]), mean([yS,yE]), sprintf('k=%d',k), ... 
%              'HorizontalAlignment','center', 'FontWeight','bold');
% 
%         % 7. Append results
%         if last_idx < s_idx
%             x_out = [x_out; x_ref(last_idx:s_idx-1)];
%             y_out = [y_out; y_ref(last_idx:s_idx-1)];
%         end
%         x_out = [x_out; bez_x]; y_out = [y_out; bez_y];
%         last_idx = e_idx + 1;
%     end
% 
%     % Finalize plot
%     legend('Original Path', 'Location', 'best');
%     axis equal; grid on;
% 
%     % Append remaining points
%     if last_idx <= length(x_ref)
%         x_out = [x_out; x_ref(last_idx:end)];
%         y_out = [y_out; y_ref(last_idx:end)];
%     end
% end
% 
% function p_clamped = clamp_control_point(origin, target, max_dist)
%     vec = target - origin;
%     dist = norm(vec);
%     if dist > max_dist
%         p_clamped = origin + vec*(max_dist/dist);
%     else
%         p_clamped = target;
%     end
% end

function [x_out, y_out] = modify_curves_bezier(x_ref, y_ref, theta_ref, curve_bounds, d1, d2, map)    
    x_ref = x_ref(:); y_ref = y_ref(:); 
    theta_ref = unwrap(theta_ref(:));
    x_out = []; y_out = [];
    n = 100; last_idx = 1;

    % Parameters for outward enhancement
    OUTWARD_FACTOR = 2;  % Increase to push curves outward more
    MAX_REACH = 0.70;      % 45% of segment length

    figure(1); clf; hold on; % Initialize debug plot
    plot(x_ref, y_ref, 'k--', 'DisplayName', 'Original Path');

    for c = 1:size(curve_bounds,1)
        s_idx = curve_bounds(c,1); e_idx = curve_bounds(c,2);
        xS = x_ref(s_idx); yS = y_ref(s_idx);
        xE = x_ref(e_idx); yE = y_ref(e_idx);
        thetaS = theta_ref(s_idx); thetaE = theta_ref(e_idx);
        seg_len = norm([xE-xS, yE-yS]);

        % 1. Curvature detection (cross product method)
        mid_idx = round(mean([s_idx, e_idx]));
        P_mid = [x_ref(mid_idx), y_ref(mid_idx)];
        segment_dir = [cos(thetaS), sin(thetaS)];
        vec_to_mid = P_mid - [xS, yS];
        cross_val = segment_dir(1)*vec_to_mid(2) - segment_dir(2)*vec_to_mid(1);
        k = sign(cross_val); % +1=left turn, -1=right turn

        % 2. Perpendicular directions (visually verified)
        right_perp = @(theta) [sin(theta), -cos(theta)] * OUTWARD_FACTOR;
        left_perp = @(theta) [-sin(theta), cos(theta)] * OUTWARD_FACTOR;

        if k > 0 % Left turn: control points on RIGHT side
            perpS = right_perp(thetaS);
            perpE = right_perp(thetaE);
            curve_color = [0 0.7 0]; % Dark green for left turns
        else % Right turn: control points on LEFT side
            perpS = left_perp(thetaS);
            perpE = left_perp(thetaE);
            curve_color = [0.7 0 0]; % Dark red for right turns
        end

        % 3. Control point generation
        d1_eff = max(d1*seg_len, 1.5);
        d2_eff = max(d2*seg_len, 1.5);
        %%%%%%%%%%%%
                % Obstacle-aware lateral expansion
        min_d2 = 0.1;               % Minimum lateral offset
        steps_d2 = 5;               % How many tries to reduce d2
        d2_eff = max(d2*seg_len, 1.5);  % Start with full offset
        
        % Check lateral clearance using occupancy map
        for s = 1:steps_d2
            % Trial control points
            trial_d2 = d2_eff;
            if k > 0
                p1_check = [xS, yS] + d1_eff*[cos(thetaS), sin(thetaS)] + trial_d2*right_perp(thetaS);
                p2_check = [xE, yE] - d1_eff*[cos(thetaE), sin(thetaE)] + trial_d2*right_perp(thetaE);
            else
                p1_check = [xS, yS] + d1_eff*[cos(thetaS), sin(thetaS)] + trial_d2*left_perp(thetaS);
                p2_check = [xE, yE] - d1_eff*[cos(thetaE), sin(thetaE)] + trial_d2*left_perp(thetaE);
            end
        
            % Check if either control point is in/near an obstacle
            if getOccupancy(map, p1_check) > 0.1 || getOccupancy(map, p2_check) > 0.1
                d2_eff = d2_eff * 0.7;  % Reduce lateral offset
                if d2_eff < min_d2
                    d2_eff = min_d2;
                    break;
                end
            else
                break;  % Safe to proceed
            end
        end
        %%%%%%%%%%%%%
        p1 = [xS, yS] + d1_eff*[cos(thetaS), sin(thetaS)] + d2_eff*perpS;
        p2 = [xE, yE] - d1_eff*[cos(thetaE), sin(thetaE)] + d2_eff*perpE;

        % 4. Clamping
        p1 = clamp_control_point([xS, yS], p1, seg_len*MAX_REACH);
        p2 = clamp_control_point([xE, yE], p2, seg_len*MAX_REACH);

        % 5. Bézier curve
        t = linspace(0,1,n)';
        bez_x = (1-t).^3*xS + 3*(1-t).^2.*t*p1(1) + 3*(1-t).*t.^2*p2(1) + t.^3*xE;
        bez_y = (1-t).^3*yS + 3*(1-t).^2.*t*p1(2) + 3*(1-t).*t.^2*p2(2) + t.^3*yE;

        % 6. Debug plotting
        plot(xS, yS, 'go', 'MarkerSize', 8, 'LineWidth', 2); % Start marker
        plot(xE, yE, 'ro', 'MarkerSize', 8, 'LineWidth', 2); % End marker
        plot(p1(1), p1(2), 'b*', 'MarkerSize', 12); % Control point 1
        plot(p2(1), p2(2), 'm*', 'MarkerSize', 12); % Control point 2
        plot([xS, p1(1)], [yS, p1(2)], 'b--'); % Control line 1
        plot([xE, p2(1)], [yE, p2(2)], 'm--'); % Control line 2
        plot(bez_x, bez_y, 'LineWidth', 2, 'Color', curve_color); % Curve itself
        text(mean([xS,xE]), mean([yS,yE]), sprintf('k=%d',k), ... 
             'HorizontalAlignment','center', 'FontWeight','bold');

        % 7. Append results
        if last_idx < s_idx
            x_out = [x_out; x_ref(last_idx:s_idx-1)];
            y_out = [y_out; y_ref(last_idx:s_idx-1)];
        end
        x_out = [x_out; bez_x]; y_out = [y_out; bez_y];
        last_idx = e_idx + 1;
    end

    % Finalize plot
    legend('Original Path', 'Location', 'best');
    axis equal; grid on;

    % Append remaining points
    if last_idx <= length(x_ref)
        x_out = [x_out; x_ref(last_idx:end)];
        y_out = [y_out; y_ref(last_idx:end)];
    end
end

function p_clamped = clamp_control_point(origin, target, max_dist)
    vec = target - origin;
    dist = norm(vec);
    if dist > max_dist
        p_clamped = origin + vec*(max_dist/dist);
    else
        p_clamped = target;
    end
end


