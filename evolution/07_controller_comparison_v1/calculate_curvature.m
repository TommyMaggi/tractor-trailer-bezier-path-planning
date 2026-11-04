function curvatures = calculate_curvature(x, y)
    % Calculate first and second derivatives numerically
    dx = gradient(x);
    dy = gradient(y);
    ddx = gradient(dx);
    ddy = gradient(dy);
    
    % Curvature formula: |x'y'' - y'x''| / ((x'^2 + y'^2)^(3/2))
    numerator = abs(dx .* ddy - dy .* ddx);
    denominator = (dx.^2 + dy.^2).^(3/2);
    
    % Avoid division by zero for straight, evenly spaced lines
    curvatures = numerator ./ (denominator + 1e-6); 
end