clc; clear; close all;

%% === Robot Parameters ===
dt = 0.05;
t_end = 1000;
time = 0:dt:t_end;
v = 0.04;     % Realistic forward velocity
L = 0.5;

%% === Trajectory Selection ===
disp("Select a trajectory to follow:");
disp("1. Sine Wave");
disp("2. Rectangle");
disp("3. Maze-like Zigzag");
disp("4. Spiral");
choice = input("Enter your choice (1-4): ");

switch choice
    case 1
        x_line = linspace(0, 20, 400);
        y_line = 2*sin(0.5*x_line) + 1.5*cos(0.3*x_line) + 0.5*sin(1.5*x_line);

    case 2
        x_line = [0:0.2:10, 10*ones(1,26), 10:-0.2:0, zeros(1,26)];
        y_line = [zeros(1,51), 0:0.2:5, 5*ones(1,51), 5:-0.2:0];
    case 3
        x_line = linspace(0, 20, 400);
        y_line = square(0.5 * x_line) + 1.5*sin(0.2*x_line);
    case 4
        t_spiral = linspace(0, 4*pi, 400);
        r = linspace(0.5, 5, 400);
        x_line = r .* cos(t_spiral) + 10;
        y_line = r .* sin(t_spiral) + 5;
    otherwise
        error('Invalid choice');
end

%% === Axes Limits ===
x_margin = 3; y_margin = 3;
x_min = min(x_line) - x_margin; x_max = max(x_line) + x_margin;
y_min = min(y_line) - y_margin; y_max = max(y_line) + y_margin;

%% === Initial Robot State ===
x = x_line(1); y = y_line(1) - 1;
theta = atan2(y_line(2) - y_line(1), x_line(2) - x_line(1));
x_hist = x; y_hist = y;

%% === Auto-tuned PID ===
plant = tf(1, [1 0]);  % Integrator
pid_autotuned = pidtune(plant, 'PID');
Kp = pid_autotuned.Kp;
Ki = pid_autotuned.Ki;
Kd = pid_autotuned.Kd;

%% === Drawing Setup ===
robotLength = 1.2; robotWidth = 0.8;
wheelLength = 0.3; wheelWidth = 0.12;
angle_error_prev = 0;
angle_error_sum = 0;
goal_hold_time = 0;

% PID Logs
angle_error_log = [];
angular_velocity_log = [];
P_log = [];
I_log = [];
D_log = [];

%% === Visualization ===
%% === Visualization ===
figure('Color','w', 'Position', [100, 100, 1000, 700]);
axis equal; grid on; hold on;
plot(x_line, y_line, 'k--', 'LineWidth', 2);  % <- Updated here
xlabel('X'); ylabel('Y');
title('Auto-Tuned PID Controlled Robot');
xlim([x_min x_max]); ylim([y_min y_max]);
robotTrail = animatedline('Color', [1.0, 0.5, 0.0], 'LineWidth', 2);  % orange


robotBody = fill(0, 0, [1 0.6 0.1]);
wheel_FL = fill(0, 0, 'k'); wheel_FR = fill(0, 0, 'k');
wheel_RL = fill(0, 0, 'k'); wheel_RR = fill(0, 0, 'k');

%% === Simulation Loop ===
for t = time
    dists = hypot(x_line - x, y_line - y);
    [~, idx] = min(dists);
    idx = max(1, min(idx + 5, length(x_line)));

    tx = x_line(idx); ty = y_line(idx);
    dx = tx - x; dy = ty - y;
    targetAngle = atan2(dy, dx);
    angleError = wrapToPi(targetAngle - theta);

    look_ahead_idx = round(10 + 5 * norm([dx dy]));
    idx = max(1, min(idx + look_ahead_idx, length(x_line)));

    % PID Calculation
    angle_error_sum = angle_error_sum + angleError * dt;
    angle_error_rate = (angleError - angle_error_prev) / dt;
    angle_error_prev = angleError;

    P_term = Kp * angleError;
    I_term = Ki * angle_error_sum;
    D_term = Kd * angle_error_rate;
    w = P_term + I_term + D_term;
    w = max(min(w, 2), -2);  % clamp

    % Logging
    angle_error_log(end+1) = angleError;
    angular_velocity_log(end+1) = w;
    P_log(end+1) = P_term;
    I_log(end+1) = I_term;
    D_log(end+1) = D_term;

    % Motion
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + w * dt;
    x_hist(end+1) = x; y_hist(end+1) = y;
    addpoints(robotTrail, x, y);

    % Draw robot body
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    corners = [-robotLength/2, -robotWidth/2;
                robotLength/2, -robotWidth/2;
                robotLength/2,  robotWidth/2;
               -robotLength/2,  robotWidth/2]';
    robotXY = R * corners + [x; y];
    set(robotBody, 'XData', robotXY(1, :), 'YData', robotXY(2, :));

    % Wheels
    % offset_x = 0.25 * robotLength;
    % offset_y = 0.5 * robotWidth + 0.01;
    % wheelRect = [-wheelLength/2, -wheelWidth/2;
    %               wheelLength/2, -wheelWidth/2;
    %               wheelLength/2,  wheelWidth/2;
    %              -wheelLength/2,  wheelWidth/2]';
    % 
    % FL = R * wheelRect + [x + cos(theta)*offset_x - sin(theta)*offset_y;
    %                       y + sin(theta)*offset_x + cos(theta)*offset_y];
    % FR = R * wheelRect + [x + cos(theta)*offset_x + sin(theta)*offset_y;
    %                       y + sin(theta)*offset_x - cos(theta)*offset_y];
    % RL = R * wheelRect + [x - cos(theta)*offset_x - sin(theta)*offset_y;
    %                       y - sin(theta)*offset_x + cos(theta)*offset_y];
    % RR = R * wheelRect + [x - cos(theta)*offset_x + sin(theta)*offset_y;
    %                       y - sin(theta)*offset_x - cos(theta)*offset_y];
    % 
    % set(wheel_FL, 'XData', FL(1,:), 'YData', FL(2,:));
    % set(wheel_FR, 'XData', FR(1,:), 'YData', FR(2,:));
    % set(wheel_RL, 'XData', RL(1,:), 'YData', RL(2,:));
    % set(wheel_RR, 'XData', RR(1,:), 'YData', RR(2,:));

    drawnow limitrate;

    % Stop condition
    if idx >= length(x_line) - 5
        dist_to_goal = hypot(x - x_line(end), y - y_line(end));
        if dist_to_goal < 0.2 && abs(angleError) < 0.1
            goal_hold_time = goal_hold_time + dt;
        else
            goal_hold_time = 0;
        end
        if goal_hold_time > 1.5
            disp("🎯 Goal Reached");
            break;
        end
    end
end

%% === Trajectory Comparison ===
figure('Color','w');
plot(x_line, y_line, 'k--', 'LineWidth', 2); hold on;
plot(x_hist, y_hist, 'r-', 'LineWidth', 2);
legend('Target Path', 'Robot Path');
xlabel('X'); ylabel('Y'); grid on;
title('Trajectory vs Robot Path');
axis equal; xlim([x_min x_max]); ylim([y_min y_max]);

%% === PID Control Plots ===
figure('Name','PID Behavior','Color','w');
subplot(3,1,1);
plot(time(1:length(angle_error_log)), angle_error_log, 'b');
ylabel('Angle Error (rad)'); grid on; title('Tracking Error');

subplot(3,1,2);
plot(time(1:length(angular_velocity_log)), angular_velocity_log, 'r');
ylabel('Angular Velocity w (rad/s)'); grid on; title('Control Command');

subplot(3,1,3);
plot(time(1:length(P_log)), P_log, 'r', ...
     time(1:length(I_log)), I_log, 'g', ...
     time(1:length(D_log)), D_log, 'b');
legend('P', 'I', 'D');
xlabel('Time (s)'); ylabel('Contribution');
title('PID Term Contributions'); grid on;
