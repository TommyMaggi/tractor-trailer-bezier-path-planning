clc; clear; close all;
%% === 1. Main Simulation & Vehicle Parameters ===
fprintf('Setting up test parameters...\n');

% --- Unified Simulation Parameters ---
sim_params.dt = 0.05;       % Time step (s), affects simulation accuracy
sim_params.t_end = 700;      % Total simulation duration (s)
sim_params.v = 0.04;         % Constant vehicle velocity (m/s)

% --- Vehicle Geometry & Scale ---
vehicle_scale_factor = 0.48; 
geom.truck_length = 2.5 * vehicle_scale_factor;
geom.trailer_length = 1.5 * vehicle_scale_factor;
geom.hitch_length = 2.0 * vehicle_scale_factor;
geom.truck_width = 1.4 * vehicle_scale_factor;
geom.trailer_width = 1.45 * vehicle_scale_factor;

% --- Controller Tuning ---
control_params.fixedLookahead = 3.0;
control_params.minLookahead = 1.5;
control_params.maxLookahead = 4.0;
control_params.curvature_gain = 100.0;
control_params.sharp_turn_threshold = 2.0;
control_params.sharp_turn_padding = 2.0;


%% === 2. Path Generation ===
disp("Select a trajectory to follow:");
disp("1. Sine Wave");
disp("2. Rectangle");
disp("3. Maze-like Zigzag");
disp("4. Spiral");
choice = input("Enter your choice (1-4): ");
switch choice
    case 1
        x_ref = linspace(0, 20, 400);
        y_ref = 2*sin(0.5*x_ref) + 1.5*cos(0.3*x_ref) + 0.5*sin(1.5*x_ref);

    case 2
        x_ref = [0:0.2:10, 10*ones(1,26), 10:-0.2:0, zeros(1,26)];
        y_ref = [zeros(1,51), 0:0.2:5, 5*ones(1,51), 5:-0.2:0];
    case 3
        x_ref = linspace(0, 20, 400);
        y_ref = square(0.5 * x_ref) + 1.5*sin(0.2*x_ref);
    case 4
        t_spiral = linspace(0, 4*pi, 400);
        r = linspace(0.5, 5, 400);
        x_ref = r .* cos(t_spiral) + 10;
        y_ref = r .* sin(t_spiral) + 5;
    otherwise
        error('Invalid choice');
end
% --- Pre-processing for Controller ---
path_curvatures = calculate_curvature(x_ref, y_ref);
avg_step_dist = mean(vecnorm(diff([x_ref(:), y_ref(:)]),2,2));
padding_indices = round(control_params.sharp_turn_padding / avg_step_dist);
sharp_turn_zones = detect_sharp_turns(path_curvatures, control_params.sharp_turn_threshold, padding_indices);


%% === 3. Run Simulations ===
fprintf('\n--- Starting Simulation Runs ---\n');
results = {}; legends = {};
base_params = struct('geom', geom, 'sim', sim_params, 'control', control_params, 'sharp_turn_zones', sharp_turn_zones);

params1 = base_params; params1.controlPoint = 'front'; params1.lookaheadType = 'fixed';
results{1} = simulate_vehicle_refactored(x_ref, y_ref, path_curvatures, params1); legends{1} = 'Control: Front, Lookahead: Fixed';
params2 = base_params; params2.controlPoint = 'rear'; params2.lookaheadType = 'fixed';
results{2} = simulate_vehicle_refactored(x_ref, y_ref, path_curvatures, params2); legends{2} = 'Control: Rear, Lookahead: Fixed';
params3 = base_params; params3.controlPoint = 'rear'; params3.lookaheadType = 'adaptive';
results{3} = simulate_vehicle_refactored(x_ref, y_ref, path_curvatures, params3); legends{3} = 'Control: Rear, Lookahead: Adaptive';

fprintf('--- All simulations complete. ---\n\n');


%% === 4. Post-Processing: Calculate Tracking Error ===
fprintf('Calculating tracking error for each run...\n');
for i = 1:length(results)
    rear_axle_x = results{i}.truck.x - (geom.truck_length/2) .* cos(results{i}.truck.theta);
    rear_axle_y = results{i}.truck.y - (geom.truck_length/2) .* sin(results{i}.truck.theta);
    results{i}.cte = calculate_cte(x_ref, y_ref, rear_axle_x, rear_axle_y);
end
fprintf('--- Error calculation complete. ---\n');


%% === 5. Generate Analysis Plots ===
fprintf('Generating analysis plots...\n');
colors = lines(length(results));

% --- Plot 1: Trajectory Comparison ---
figure('Name', 'Trajectory Comparison');
plot(x_ref, y_ref, 'k--', 'LineWidth', 2, 'DisplayName', 'Reference Path');
hold on; grid on; axis equal;
for i = 1:length(results)
    plot(results{i}.truck.x, results{i}.truck.y, '-', 'Color', colors(i,:), 'DisplayName', ['Truck: ' legends{i}]);
    plot(results{i}.trailer.x, results{i}.trailer.y, ':', 'Color', colors(i,:), 'HandleVisibility', 'off');
end
title('Trajectory Following Performance'); xlabel('X (m)'); ylabel('Y (m)');
legend('show', 'Location', 'best'); hold off;

% --- Plot 2: Comprehensive Controller Analysis ---
figure('Name', 'Controller Behavior Analysis');
ax1 = subplot(3,1,1);
hold on; grid on;
for i = 1:length(results)
    plot(ax1, results{i}.time, results{i}.cte, '-', 'Color', colors(i,:));
end
title(ax1, 'Cross-Track Error (CTE)'); ylabel(ax1, 'Error (m)'); legend(ax1, legends);

ax2 = subplot(3,1,2);
hold on; grid on;
for i = 1:length(results)
    plot(ax2, results{i}.time, rad2deg(results{i}.steering_hist), '-', 'Color', colors(i,:));
end
title(ax2, 'Steering Angle Command (\delta)'); ylabel(ax2, 'Angle (degrees)');

ax3 = subplot(3,1,3);
hold on; grid on;
for i = 1:length(results)
    plot(ax3, results{i}.time, results{i}.lookahead_hist, '-', 'Color', colors(i,:));
end
title(ax3, 'Adaptive Lookahead Distance'); xlabel(ax3, 'Time (s)'); ylabel(ax3, 'Distance (m)');

fprintf('--- Analysis plots complete. ---\n');


%% === 6. Synchronized Animation ===
fprintf('\n--- Animation Selection ---\n');
for i = 1:length(legends)
    fprintf('%d: %s\n', i, legends{i});
end
prompt = 'Enter the plot numbers to animate (e.g., 1 3 or 2): ';
user_choice_str = input(prompt, 's');
plot_indices = str2num(user_choice_str);

if isempty(plot_indices) || any(plot_indices > length(results)) || any(plot_indices < 1)
    fprintf('Invalid selection. Skipping animation.\n');
    return;
end

fprintf('\nGenerating synchronized animations for selection: %s ...\n', num2str(plot_indices));

figure('Name', 'Synchronized Animation Comparison', 'Position', [50, 50, 600 * length(plot_indices), 600]);
anim_objects = {}; 
for i = 1:length(plot_indices)
    ax = subplot(1, length(plot_indices), i);
    current_idx = plot_indices(i);
    
    plot(ax, x_ref, y_ref, 'k--', 'LineWidth', 1.5);
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    
    anim_objects{i}.truck_patch = patch(ax, 'Vertices', zeros(4,2), 'Faces', 1:4, 'FaceColor', 'blue', 'FaceAlpha', 0.8);
    anim_objects{i}.trailer_patch = patch(ax, 'Vertices', zeros(4,2), 'Faces', 1:4, 'FaceColor', 'red', 'FaceAlpha', 0.8);
    anim_objects{i}.hitch_line = plot(ax, [0 0], [0 0], 'k-', 'LineWidth', 2);
    
    title(ax, legends{current_idx});
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
    xlim(ax, [min(x_ref)-5, max(x_ref)+5]); ylim(ax, [min(y_ref)-5, max(y_ref)+5]);
end
sgtitle('Synchronized Controller Performance', 'FontSize', 16, 'FontWeight', 'bold');

selected_results = results(plot_indices);
min_len = min(cellfun(@(r) length(r.truck.x), selected_results));
animation_step = 5;

for i = 1:animation_step:min_len
    for j = 1:length(plot_indices)
        current_idx = plot_indices(j);
        
        truck = results{current_idx}.truck;
        trailer = results{current_idx}.trailer;
        
        truck_rect = get_rectangle(truck.x(i), truck.y(i), truck.theta(i), geom.truck_length, geom.truck_width);
        set(anim_objects{j}.truck_patch, 'Vertices', truck_rect);

        trailer_rect = get_rectangle(trailer.x(i), trailer.y(i), trailer.theta(i), geom.trailer_length, geom.trailer_width);
        set(anim_objects{j}.trailer_patch, 'Vertices', trailer_rect);

        hitch_x = truck.x(i) - (geom.truck_length/2)*cos(truck.theta(i));
        hitch_y = truck.y(i) - (geom.truck_length/2)*sin(truck.theta(i));
        trailer_hitch_x = trailer.x(i) + (geom.trailer_length/2)*cos(trailer.theta(i));
        trailer_hitch_y = trailer.y(i) + (geom.trailer_length/2)*sin(trailer.theta(i));
        set(anim_objects{j}.hitch_line, 'XData', [hitch_x trailer_hitch_x], 'YData', [hitch_y trailer_hitch_y]);
    end
    sgtitle(sprintf('Synchronized Controller Performance - Time: %.2f s', results{1}.time(i)), 'FontSize', 16, 'FontWeight', 'bold');
    
    drawnow limitrate; % MODIFIED: Replaced pause(dt) with drawnow limitrate for faster animation
end
fprintf('--- Synchronized animation finished. ---\n');

function rect = get_rectangle(x, y, theta, L, W)
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    corners = [-L/2 -W/2; L/2 -W/2; L/2 W/2; -L/2 W/2];
    rect = (R * corners')' + [x y];
end