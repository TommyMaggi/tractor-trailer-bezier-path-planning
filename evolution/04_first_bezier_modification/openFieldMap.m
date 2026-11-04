% openField = binaryOccupancyMap(10,10,5);
% walls = zeros(50,50);
% walls(1,:) = 1;% Top wall
% walls(end,:) = 1; % Bottom wall
% walls(:,1) = 1; % Left wall
% walls(:,end) = 1; % Right wall
% walls(1:30,15) = 1; % Left division
% walls(25:end,25) = 1; % Middle division
% walls(1:30,35) = 1; % Right division
% 
% setOccupancy(openField,[1 1],walls,"grid")
% show(openField)
% 
% inflateRadius = 0.15; %change value of inlation
% inflate(openField,inflateRadius);
% 
% show(openField)
% Occupied = checkOccupancy(openField,[5 5]);c

topView = imread('Obstacles.png'); %top view = immagine
%imshow(topView); %mostra immagine

imageMask = createImageMask(topView); %imagemask => funzione da applicazione segmentation
imageMaskResized = imresize(imageMask,[333,500]); %resizing della maschera 666 righe (y) 1000 colonne (x)
%imshow(imageMaskResized);

openField = binaryOccupancyMap(50,33.3,10); %100 mt * 66 mt -> 10 celle per metro imim
setOccupancy(openField,[1 1],~imageMaskResized,"grid"); %openfield con valori di imagemask resized

%show(openField)

planningMap = copy(openField);

inflateRadius = 0.75; % meters, change as needed
inflate(planningMap, inflateRadius);
figure;
show(openField); hold on;
title('Inflated Map with Obstacle Buffer'); hold off;

%prmComplex = mobileRobotPRM(openField,25);
%show(prmComplex)

N = 250; % number of nodes

% Get map world limits
xLimits = openField.XWorldLimits; % [0, 100]
yLimits = openField.YWorldLimits; % [0, 66.6]

% Initialize node list
nodes = [];
maxTries = 10000;
attempts = 0;

while size(nodes, 1) < N && attempts < maxTries
    % Sample in WORLD coordinates (meters)
    x = rand() * (xLimits(2) - xLimits(1));
    y = rand() * (yLimits(2) - yLimits(1));
    attempts = attempts + 1;

    % Check occupancy at this world location
    if getOccupancy(planningMap, [x, y]) == 0
        nodes(end+1, :) = [x, y]; % Save in world coordinates
    end
end

% Report if something went wrong
if size(nodes, 1) < N
    warning('Only %d free nodes found after %d attempts.', size(nodes,1), attempts);
end

% === Visualization ===
figure;
show(planningMap); hold on;
plot(nodes(:,1), nodes(:,2), 'b.', 'MarkerSize', 10);
title('Step 1: Sampled Free Nodes in World Coordinates');
xlabel('X (m)');
ylabel('Y (m)');
axis equal;

connectionRadius = 5; %8 meters
numNodes = size(nodes,1);

adjacency = zeros(numNodes); %relative distances among points

for i = 1:numNodes
    for j = i+1:numNodes
        % Get both points
        p1 = nodes(i, :);
        p2 = nodes(j, :);

        % Check distance
        dist = norm(p1 - p2);
        if dist <= connectionRadius
            % Check collision along the line between p1 and p2
            if isCollisionFree(planningMap, p1, p2)
                % Mark connection (undirected graph)
                adjacency(i, j) = dist;
                adjacency(j, i) = dist;

                % OPTIONAL: visualize connection
                plot([p1(1), p2(1)], [p1(2), p2(2)], 'g-', 'LineWidth', 0.5);
            end
        end
    end
end

title('Step 2: PRM Roadmap Connections');

% %% === STEP 2: PRM roadmap - semi-built-in version (vectorized distance) ===
% connectionRadius = 8; % meters
% numNodes = size(nodes, 1);
% adjacency = zeros(numNodes);
% 
% % Compute pairwise distances using MATLAB built-in pdist2
% distMatrix = pdist2(nodes, nodes); % NxN matrix of Euclidean distances
% 
% for i = 1:numNodes
%     for j = i+1:numNodes
%         dist = distMatrix(i, j);
%         if dist <= connectionRadius
%             if isCollisionFree(openField, nodes(i,:), nodes(j,:))
%                 adjacency(i,j) = dist;
%                 adjacency(j,i) = dist;
%                 plot([nodes(i,1), nodes(j,1)], [nodes(i,2), nodes(j,2)], 'g-', 'LineWidth', 0.5);
%             end
%         end
%     end
% end
% 
% % % title('Step 2: PRM Roadmap (with pdist2)');

%% === STEP 3: Add start and goal nodes manually ===
start = [2.5, 2.5];     % You can change these
goal = [45, 30];

nodes = [start; goal; nodes];  % now: node 1 = start, node 2 = goal

numNodes = size(nodes, 1);

% Expand the adjacency matrix to include start and goal
adjacency = [zeros(2, numNodes); zeros(numNodes-2, 2), adjacency];

% Connect start and goal to nearby nodes
for i = 3:numNodes
    for j = 1:2 % for start and goal
        dist = norm(nodes(i,:) - nodes(j,:));
        if dist <= connectionRadius
            if isCollisionFree(planningMap, nodes(i,:), nodes(j,:))
                adjacency(i, j) = dist;
                adjacency(j, i) = dist;

                % plot new edges
                plot([nodes(i,1), nodes(j,1)], [nodes(i,2), nodes(j,2)], 'r--');
            end
        end
    end
end

%% === STEP 4: Dijkstra (manual version) ===
startIdx = 1;
goalIdx = 2;

visited = false(1, numNodes);
dist = inf(1, numNodes);
prev = nan(1, numNodes);
dist(startIdx) = 0;

for k = 1:numNodes
    % Find unvisited node with smallest distance
    [~, u] = min(dist + visited*1e6); 
    visited(u) = true;

    neighbors = find(adjacency(u, :) > 0);
    for v = neighbors
        alt = dist(u) + adjacency(u, v);
        if alt < dist(v)
            dist(v) = alt;
            prev(v) = u;
        end
    end

    if u == goalIdx
        break; % we reached the goal
    end
end

% Reconstruct path
path = goalIdx;
while prev(path(1)) ~= startIdx && ~isnan(prev(path(1)))
    path = [prev(path(1)), path];
end
path = [startIdx, path];

% Plot final path
for i = 1:length(path)-1
    p1 = nodes(path(i), :);
    p2 = nodes(path(i+1), :);
    plot([p1(1), p2(1)], [p1(2), p2(2)], 'r-', 'LineWidth', 2);
end
title('Final Path with Dijkstra (manual)');
%% 

x_ref = nodes(path, 1);
y_ref = nodes(path, 2);

% Step 0: Start from PRM path (sparse!)
original_path = [x_ref, y_ref]; % maybe only 20–30 points

% Step 1: Compute cumulative arc-length (parameterization)
segment_lengths = vecnorm(diff(original_path,1,1), 2, 2);
cum_dist = [0; cumsum(segment_lengths)];

% Step 2: Dense interpolation
num_interp_points = 2500; % you can lower to 1000 if needed
interp_dist = linspace(0, cum_dist(end), num_interp_points);

x_ref = interp1(cum_dist, x_ref, interp_dist, 'pchip');
y_ref = interp1(cum_dist, y_ref, interp_dist, 'pchip');

% % Optional: try without these first
% x_ref = smoothdata(x_ref, 'gaussian', 10);
% y_ref = smoothdata(y_ref, 'gaussian', 10);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dx = diff(x_ref);
dy = diff(y_ref);
theta_ref = atan2(dy, dx);
theta_ref = smoothdata(theta_ref, 'gaussian', 10);
theta_ref = theta_ref(:); % force column
theta_ref = [theta_ref; theta_ref(end)];

dt = 0.05;
vx = sqrt(dx.^2 + dy.^2) / dt;
vx = vx(:);
vx = [vx; vx(end)];  % pad to match theta_ref
omega = [0; diff(theta_ref)] / dt;

% Thresholds
vx_threshold = 0.02;
omega_threshold = deg2rad(3);
min_curve_length = 30;  % ← elimina curve troppo corte
min_gap = 50;           % ← unisci curve troppo vicine
padding = 20;            % ← extra margine a inizio/fine curva

% --- Detect curves ---
is_curve = (abs(vx) > vx_threshold) & (abs(omega) > omega_threshold);

curve_bounds = [];
in_curve = false;
for i = 1:length(is_curve)
    if is_curve(i) && ~in_curve
        start_idx = i;
        in_curve = true;
    elseif ~is_curve(i) && in_curve
        end_idx = i - 1;
        in_curve = false;

        % Filtro lunghezza minima
        if (end_idx - start_idx) >= min_curve_length
            curve_bounds = [curve_bounds; start_idx, end_idx];
        end
    end
end

% --- Merge curve segments too close together ---
merged_bounds = [];
i = 1;
while i <= size(curve_bounds,1)
    s_idx = curve_bounds(i,1);
    e_idx = curve_bounds(i,2);

    while i < size(curve_bounds,1) && ...
          curve_bounds(i+1,1) - e_idx < min_gap
        i = i + 1;
        e_idx = curve_bounds(i,2);
    end

    merged_bounds = [merged_bounds; s_idx, e_idx];
    i = i + 1;
end

% --- Add padding ---
curve_bounds_padded = [];
for i = 1:size(merged_bounds,1)
    start_idx = max(1, merged_bounds(i,1) - padding);
    end_idx   = min(length(x_ref), merged_bounds(i,2) + padding);
    curve_bounds_padded = [curve_bounds_padded; start_idx, end_idx];
end
figure;
    plot(x_ref, y_ref, 'k--'); hold on; axis equal; title('Curves Detected');

for i = 1:size(curve_bounds_padded,1)
    idx_range = curve_bounds_padded(i,1):curve_bounds_padded(i,2);
    plot(x_ref(idx_range), y_ref(idx_range), 'm-', 'LineWidth', 2);
end
legend('Reference Path', 'Detected Curves');

figure;
subplot(3,1,1);
plot(theta_ref); title('θ\_ref');
subplot(3,1,2);
plot(omega); title('ω (angular velocity)');
subplot(3,1,3);
plot(vx); title('v\_x');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%


[truck, trailer, target_xs, target_ys] = simulate_vehicle(x_ref, y_ref);


figure('Name','Final Trajectory Visualization');
show(openField); hold on;
plot(x_ref, y_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Path');
plot(truck.x, truck.y, 'b-', 'LineWidth', 2, 'DisplayName', 'Truck Path');
plot(trailer.x, trailer.y, 'r-', 'LineWidth', 2, 'DisplayName', 'Trailer Path');
legend('Location', 'best');
xlabel('X (m)');
ylabel('Y (m)');
axis equal;
title('Final Path Visualization');




animate_vehicles_with_map(openField, x_ref, y_ref, truck, trailer, target_xs, target_ys);

d1 = 1.5;  % along-path control point distance (adjust as needed)
d2 = 0.75; % lateral safety margin (adjust as needed)

%[x_ref_mod, y_ref_mod] = modify_curves_bezier(x_ref, y_ref, theta_ref, curve_bounds_padded, d1, d2);
[x_ref_mod, y_ref_mod] = modify_curves_bezier(x_ref, y_ref, theta_ref, curve_bounds_padded, d1, d2, openField);
% [x_ref_mod, y_ref_mod] = plan_bezier_with_feedback(...
%     x_ref, y_ref, theta_ref, curve_bounds_padded, ...
%     openField, d1, 0.75, 0.2, 2.0, 35);

% [x_ref_mod, y_ref_mod] = plan_bezier_dual_d2(...
%     x_ref, y_ref, theta_ref, curve_bounds_padded, ...
%     openField, d1, 0.75, 35);

% d2_init, d2_min, d2_max, pivot_threshold
% Visualize it
figure;
plot(x_ref, y_ref, 'k--', 'DisplayName', 'Original Path'); hold on;
plot(x_ref_mod, y_ref_mod, 'b-', 'LineWidth', 2, 'DisplayName', 'Modified Path');
legend('Location', 'best');
axis equal;
title('Recker-style Bézier Path Modification');

%plot_results(truck, trailer);
