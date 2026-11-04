function sharp_turn_zones = detect_sharp_turns(path_curvatures, curvature_threshold, padding_indices)
    % Initialize a boolean array of the same size as the path, all set to false.
    sharp_turn_zones = false(size(path_curvatures));
    
    % Find all points where the curvature exceeds the threshold.
    sharp_indices = find(path_curvatures > curvature_threshold);
    
    % If any sharp points are found, create padded zones around them.
    if ~isempty(sharp_indices)
        fprintf('  Detected %d high-curvature points. Creating padded zones...\n', length(sharp_indices));
        for i = 1:length(sharp_indices)
            idx = sharp_indices(i);
            
            % Define the start and end of the padded zone for this point.
            start_zone = max(1, idx - padding_indices);
            end_zone = min(length(path_curvatures), idx + padding_indices);
            
            % Set the elements within this zone to 'true'.
            sharp_turn_zones(start_zone:end_zone) = true;
        end
    end
end