% function plot_results(truck, trailer)
%     % Mutual angle plot
%     figure('Name','Mutual Angle');
%     time = (0:length(truck.theta)-1) * 0.1; % dt=0.1
%     mutual_angle = rad2deg(truck.theta - trailer.theta);
%     plot(time, mutual_angle, 'm', 'LineWidth', 2);
%     ylabel('Angle (deg)'); xlabel('Time (s)');
%     title('Truck-Trailer Mutual Angle');
%     grid on;
% 
%     % Absolute orientations plot
%     figure('Name','Absolute Orientations');
%     plot(time, rad2deg(truck.theta), 'b', 'LineWidth', 2); hold on;
%     plot(time, rad2deg(trailer.theta), 'r', 'LineWidth', 2);
%     legend('Truck', 'Trailer');
%     ylabel('Angle (deg)'); xlabel('Time (s)');
%     title('Absolute Orientations');
%     grid on;
% 
% 
%     %distance = sqrt((truck.x - trailer.x).^2 + (truck.y - trailer.y).^2);
% 
%     distance = sqrt((truck.x - trailer.x).^2 + (truck.y - trailer.y).^2);
% 
% % Plot the distance over time
% figure;
% plot(time, distance, 'b', 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Distance (m)');
% title('Truck-Trailer Distance Over Time');
% grid on;
% end
% 
function plot_results(truck, trailer)
    % Process angles to remove discontinuities
    time = (0:length(truck.theta)-1) * 0.1; % Assuming dt=0.1
    
    % Unwrap angles before conversion to degrees
    truck_theta = unwrap(truck.theta);
    trailer_theta = unwrap(trailer.theta);
    mutual_angle = unwrap(truck.theta - trailer.theta);
    
    % Convert to degrees after unwrapping
    truck_deg = rad2deg(truck_theta);
    trailer_deg = rad2deg(trailer_theta);
    mutual_deg = rad2deg(mutual_angle);

    % Plot 1: Mutual Angle
    figure('Name','Truck-Trailer Mutual Angle');
    plot(time, mutual_deg, 'm', 'LineWidth', 2);
    title('Truck-Trailer Mutual Angle');
    xlabel('Time (s)');
    ylabel('Angle Difference (deg)');
    grid on;
    ylim([-180 180]); % Maintain -180 to 180 range for interpretability
    yticks(-180:45:180);
    
    % Plot 2: Absolute Orientations
    figure('Name','Absolute Orientations');
    plot(time, truck_deg, 'b', 'LineWidth', 2);
    hold on;
    plot(time, trailer_deg, 'r', 'LineWidth', 2);
    title('Absolute Orientations');
    xlabel('Time (s)');
    ylabel('Orientation (deg)');
    legend('Truck', 'Trailer', 'Location', 'best');
    grid on;
    
    % Optional: Add secondary axis showing unwrapped angles
    yyaxis right
    plot(time, truck_theta, 'b:', 'LineWidth', 1);
    plot(time, trailer_theta, 'r:', 'LineWidth', 1);
    ylabel('Unwrapped Angle (rad)');
    legend('Truck (unwrapped)', 'Trailer (unwrapped)', 'Location', 'southwest');
    yyaxis left
end