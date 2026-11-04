%% SCRIPT FOR ADVANCED CONTROLLER COMPARISON (THESIS CHAPTER 4) - V7 (Flexible Multi-Compare)
%
% This script provides a flexible framework for simulating and comparing
% multiple path tracking controllers.
%
% SIMULATION MODES:
% 1. Single Controller Run: Configure and simulate one controller.
% 2. Comparative Run: Configure and simulate TWO or THREE controller setups
%    (e.g., A vs. B vs. C) for direct comparison.
%
clc; clear; close all;

%% === Top-Level Mode Selection ===
disp("Select Simulation Mode:");
disp("1. Single Controller Simulation");
disp("2. Comparative Run");
sim_mode = input("Enter your choice (1 or 2): ");

%% === Trajectory Selection and Path Pre-computation ===
disp("Select a trajectory to follow:");
disp("1. Sine Wave"); disp("2. Rectangle"); disp("3. Maze-like Zigzag");
disp("4. Spiral"); disp("5. Step Response (Horizontal Line)");
choice = input("Enter your choice (1-5): ");

switch choice
    case 1, x_line=linspace(0,40,800); y_line=4*sin(0.3*x_line)+2*cos(0.15*x_line);
    case 2, x_line=[0:0.1:20,20*ones(1,101),20:-0.1:0,zeros(1,101)]; y_line=[zeros(1,201),0:0.1:10,10*ones(1,201),10:-0.1:0];
    case 3, x_line=linspace(0,40,800); y_line=5*square(0.2*x_line)+2*sin(0.1*x_line);
    case 4, t_s=linspace(0,6*pi,800); r=linspace(0.5,10,800); x_line=r.*cos(t_s)+10; y_line=r.*sin(t_s)+10;
    case 5, x_line=linspace(0,40,800); y_line=10*ones(size(x_line));
    otherwise, error('Invalid choice');
end

dx=gradient(x_line); dy=gradient(y_line); ddx=gradient(dx); ddy=gradient(dy);
curvature=abs(dx.*ddy-dy.*ddx)./((dx.^2+dy.^2+1e-6).^(3/2));
curvature_smooth = smoothdata(curvature, 'gaussian', 40);

if choice == 5, x_start=2; y_start=2; theta_start=pi/2;
else, x_start=x_line(1); y_start=y_line(1)-1.0; theta_start=atan2(y_line(2)-y_line(1),x_line(2)-x_line(1)); end

x_margin=5; y_margin=5; x_min=min([x_line,x_start])-x_margin; x_max=max([x_line,x_start])+x_margin;
y_min=min([y_line,y_start])-y_margin; y_max=max([y_line,y_start])+y_margin;

% =========================================================================
%% === MAIN LOGIC: Choose between Single Run or Comparative Run ===
% =========================================================================

if sim_mode == 1 % --- SINGLE CONTROLLER SIMULATION ---
    
    fprintf('\n--- Configuring Single Controller ---\n');
    config = configure_controller('Single');
    
    results = run_simulation(config, x_start, y_start, theta_start, ...
                             x_line, y_line, curvature_smooth);
    
    plot_single_run_results(x_line, y_line, results, config, ...
                            x_min, x_max, y_min, y_max);

else % --- FLEXIBLE COMPARATIVE SIMULATION ---
    
    num_controllers = input('\nHow many controllers to compare? (2 or 3): ');
    if num_controllers ~= 2 && num_controllers ~= 3
        error('Invalid number. Please choose 2 or 3.');
    end
    
    all_configs = cell(1, num_controllers);
    all_results = cell(1, num_controllers);
    
    for i = 1:num_controllers
        fprintf('\n--- Configuring Controller %d ---\n', i);
        all_configs{i} = configure_controller(sprintf('%d', i));
        
        fprintf('\n--- Running Simulation %d: %s ---\n', i, all_configs{i}.name);
        all_results{i} = run_simulation(all_configs{i}, x_start, y_start, theta_start, ...
                                        x_line, y_line, curvature_smooth);
    end

    plot_multi_comparative_results(x_line, y_line, all_results, all_configs, ...
                                   x_min, x_max, y_min, y_max);
end

% =========================================================================
%% === CORE SIMULATION AND CONFIGURATION FUNCTIONS ===
% =========================================================================
function config = configure_controller(id_str)
    fprintf('--- Configure Controller %s ---\n', id_str);
    disp("Select Controller Type:");
    disp("1. PID Controller");
    disp("2. Pure Pursuit (Fixed Lookahead)");
    disp("3. Pure Pursuit (Adaptive Lookahead)");
    choice = input("Enter type (1-3): ");
    
    config = struct();
    config.type = choice;
    
    switch choice
        case 1, config.name = 'PID';
        case 2
            lookahead = input('Enter FIXED lookahead distance (e.g., 3.0): ');
            config.name = sprintf('PP (L=%.1fm)', lookahead);
            config.lookahead = lookahead;
        case 3
            min_L = input('Enter MIN lookahead (e.g., 1.5): ');
            max_L = input('Enter MAX lookahead (e.g., 4.5): ');
            scale = input('Enter curvature scaling factor (e.g., 5): ');
            config.name = sprintf('Adaptive PP (L=%.1f-%.1f)', min_L, max_L);
            config.adaptive_params = struct('min', min_L, 'max', max_L, 'scale', scale);
        otherwise, error('Invalid controller type');
    end
end

function results = run_simulation(config, x_start, y_start, theta_start, ...
                                  x_line, y_line, curvature_smooth)
    dt=0.05; v=1.75; wheelbase=2.5; x=x_start; y=y_start; theta=theta_start;
    time_log=[0]; x_hist=[x]; y_hist=[y];
    [init_err,init_idx]=calculate_lateral_error(x,y,x_line,y_line);
    lat_err_log=[init_err]; ang_vel_log=[0]; extra_logs=struct();

    if config.type == 1 % PID
        plant=tf(1,[1 0]); pid=pidtune(plant,'PID'); Kp=pid.Kp; Ki=pid.Ki; Kd=pid.Kd;
        angle_err_prev=0; angle_err_sum=0;
        t_idx=min(init_idx+10,length(x_line)); tx=x_line(t_idx); ty=y_line(t_idx);
        t_ang=atan2(ty-y,tx-x); init_ang_err=wrapToPi(t_ang-theta);
        extra_logs.angle_error=[init_ang_err]; extra_logs.P=[Kp*init_ang_err];
        extra_logs.I=[0]; extra_logs.D=[0];
    elseif config.type == 3 % Adaptive PP
        % ** BUG FIX IS HERE: Initialize the log with the t=0 value **
        init_curv = curvature_smooth(init_idx);
        init_ratio = min(1.0, init_curv * config.adaptive_params.scale);
        init_lookahead = config.adaptive_params.max - (config.adaptive_params.max - config.adaptive_params.min) * init_ratio;
        extra_logs.lookahead = [init_lookahead];
    end
    
    for t_current = 0:dt:2000
        [~, closest_idx] = calculate_lateral_error(x, y, x_line, y_line);
        if config.type == 1 % PID
            t_idx=min(closest_idx+10,length(x_line)); tx=x_line(t_idx); ty=y_line(t_idx);
            t_ang=atan2(ty-y,tx-x); ang_err=wrapToPi(t_ang-theta);
            angle_err_sum=angle_err_sum+ang_err*dt; ang_err_rate=(ang_err-angle_err_prev)/dt; angle_err_prev=ang_err;
            P=Kp*ang_err; I=Ki*angle_err_sum; D=Kd*ang_err_rate; w=P+I+D;
            extra_logs.angle_error(end+1)=ang_err; extra_logs.P(end+1)=P; extra_logs.I(end+1)=I; extra_logs.D(end+1)=D;
        else % Pure Pursuit (Fixed or Adaptive)
            if config.type == 2, current_lookahead = config.lookahead;
            else, curv=curvature_smooth(closest_idx); ratio=min(1.0,curv*config.adaptive_params.scale);
                current_lookahead = config.adaptive_params.max - (config.adaptive_params.max - config.adaptive_params.min) * ratio;
                extra_logs.lookahead(end+1) = current_lookahead;
            end
            t_idx=find_lookahead_point(x,y,x_line,y_line,current_lookahead);
            tx=x_line(t_idx); ty=y_line(t_idx); alpha=atan2(ty-y,tx-x)-theta;
            delta=atan2(2*wheelbase*sin(alpha),current_lookahead); w=(v/wheelbase)*tan(delta);
        end
        w=max(min(w,2),-2);
        x=x+v*cos(theta)*dt; y=y+v*sin(theta)*dt; theta=wrapToPi(theta+w*dt);
        x_hist(end+1)=x; y_hist(end+1)=y; time_log(end+1)=t_current+dt;
        [lat_err,~]=calculate_lateral_error(x,y,x_line,y_line);
        lat_err_log(end+1)=lat_err; ang_vel_log(end+1)=w;
        if hypot(x-x_line(end),y-y_line(end))<0.5 && closest_idx>=length(x_line)-5, disp("Goal Reached"); break; end
    end
    results = struct('x_hist',x_hist,'y_hist',y_hist,'time_log',time_log,'lat_err_log',lat_err_log,'ang_vel_log',ang_vel_log,'extra_logs',extra_logs);
end

% =========================================================================
%% === PLOTTING AND HELPER FUNCTIONS ===
% =========================================================================
function plot_single_run_results(x_line, y_line, results, config, x_min, x_max, y_min, y_max)
    figure('Color','w'); plot(x_line,y_line,'k--','LineWidth',2); hold on;
    plot(results.x_hist,results.y_hist,'r-','LineWidth',2); legend('Target','Robot');
    xlabel('X (m)'); ylabel('Y (m)'); grid on; title(['Trajectory vs Robot Path: ' config.name]);
    axis equal; xlim([x_min x_max]); ylim([y_min y_max]);

    num_subplots = 2; if config.type == 1 || config.type == 3, num_subplots = 3; end
    figure('Name',['Performance Metrics: ' config.name],'Color','w');
    subplot(num_subplots,1,1); plot(results.time_log,results.lat_err_log,'b'); ylabel('Lateral Error (m)');
    grid on; title('Tracking Accuracy'); rms_err=rms(results.lat_err_log); max_err=max(abs(results.lat_err_log));
    legend(sprintf('RMS Error: %.3f m, Max Error: %.3f m', rms_err, max_err));
    subplot(num_subplots,1,2); plot(results.time_log,results.ang_vel_log,'r');
    ylabel('Angular Velocity (rad/s)'); grid on; title('Control Command Effort');
    
    if config.type == 1
        subplot(num_subplots,1,3); plot(results.time_log,results.extra_logs.P,'r',...
            results.time_log,results.extra_logs.I,'g',results.time_log,results.extra_logs.D,'b');
        legend('P','I','D'); xlabel('Time (s)'); ylabel('Contribution'); title('PID Term Contributions'); grid on;
    elseif config.type == 3
        subplot(num_subplots,1,3); plot(results.time_log, results.extra_logs.lookahead, 'm', 'LineWidth', 1.5);
        grid on; xlabel('Time (s)'); ylabel('Lookahead (m)'); title('Adaptive Lookahead Distance');
    end
end

function plot_multi_comparative_results(x_line, y_line, all_results, all_configs, x_min, x_max, y_min, y_max)
    colors = ['r', 'b', 'g'];
    line_styles = {'-', '-', '-'};
    
    figure('Name','Comparative Trajectory Tracking','Color','w');
    plot(x_line,y_line,'k--','LineWidth',2); hold on;
    legend_entries = {'Reference Path'};
    for i = 1:length(all_results)
        plot(all_results{i}.x_hist, all_results{i}.y_hist, 'Color', colors(i), 'LineStyle', line_styles{i}, 'LineWidth', 1.5);
        legend_entries{end+1} = all_configs{i}.name;
    end
    legend(legend_entries); xlabel('X (m)'); ylabel('Y (m)'); grid on;
    title('Comparative Path Tracking'); axis equal; xlim([x_min x_max]); ylim([y_min y_max]);
    
    figure('Name','Comparative Performance Metrics','Color','w');
    subplot(2,1,1); hold on;
    for i = 1:length(all_results)
        plot(all_results{i}.time_log, all_results{i}.lat_err_log, 'Color', colors(i), 'LineStyle', line_styles{i}, 'LineWidth', 1.5);
    end
    grid on; ylabel('Lateral Error (m)'); title('Comparative Tracking Accuracy'); legend(legend_entries{2:end});
    
    subplot(2,1,2); hold on;
    for i = 1:length(all_results)
        plot(all_results{i}.time_log, all_results{i}.ang_vel_log, 'Color', colors(i), 'LineStyle', line_styles{i}, 'LineWidth', 1.5);
    end
    grid on; xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
    title('Comparative Control Command Effort'); legend(legend_entries{2:end});
end

function [error, idx] = calculate_lateral_error(x, y, x_line, y_line)
    dists_sq=(x_line-x).^2+(y_line-y).^2; [min_dist_sq,idx]=min(dists_sq);
    if idx<length(x_line), p_dx=x_line(idx+1)-x_line(idx); p_dy=y_line(idx+1)-y_line(idx);
    else, p_dx=x_line(idx)-x_line(idx-1); p_dy=y_line(idx)-y_line(idx-1); end
    v_path_to_robot_x=x-x_line(idx); v_path_to_robot_y=y-y_line(idx);
    cross_prod_z=p_dx*v_path_to_robot_y-p_dy*v_path_to_robot_x;
    error=sign(cross_prod_z)*sqrt(min_dist_sq); if isnan(error),error=0;end
end
function target_idx=find_lookahead_point(x,y,x_line,y_line,Lf)
    dists_sq=(x_line-x).^2+(y_line-y).^2; [~,start_idx]=min(dists_sq);
    total_dist=0; target_idx=start_idx;
    for i=start_idx:length(x_line)-1
        seg_dist=hypot(x_line(i+1)-x_line(i),y_line(i+1)-y_line(i));
        if total_dist+seg_dist>=Lf, target_idx=i+1; return; end
        total_dist=total_dist+seg_dist;
    end
    target_idx=length(x_line);
end