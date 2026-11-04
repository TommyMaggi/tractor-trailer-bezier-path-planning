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

topView = imread('mapOpenField.png'); %top view = immagine
%imshow(topView); %mostra immagine

imageMask = createImageMask(topView); %imagemask => funzione da applicazione segmentation
imageMaskResized = imresize(imageMask,[333,500]); %resizing della maschera 666 righe (y) 1000 colonne (x)
%imshow(imageMaskResized);

openField = binaryOccupancyMap(50,33.3,10); %100 mt * 66 mt -> 10 celle per metro imim
setOccupancy(openField,[1 1],~imageMaskResized,"grid"); %openfield con valori di imagemask resized

%show(openField)

planningMap = copy(openField);

inflateRadius = 1.5; % meters, change as needed
inflate(planningMap, inflateRadius);
figure;
show(openField); show(planningMap); hold on;
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

connectionRadius = 4; %8 meters
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

x_ref = nodes(path, 1);
y_ref = nodes(path, 2);

% Interpolate to get smoother, denser trajectory
original_path = [x_ref, y_ref];
distance_between = vecnorm(diff(original_path,1,1), 2, 2);
cum_dist = [0; cumsum(distance_between)];
num_interp_points = 10000;

x_ref = interp1(cum_dist, x_ref, linspace(0, cum_dist(end), num_interp_points));
y_ref = interp1(cum_dist, y_ref, linspace(0, cum_dist(end), num_interp_points));

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

plot_results(truck, trailer);
