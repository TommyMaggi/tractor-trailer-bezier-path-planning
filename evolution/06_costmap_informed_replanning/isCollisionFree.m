function isFree = isCollisionFree(map, p1, p2)
%ISCOLLISIONFREE Checks if the path from p1 to p2 is free of obstacles
% Uses discrete interpolation and occupancy checking

    numSteps = 100;
    x = linspace(p1(1), p2(1), numSteps);
    y = linspace(p1(2), p2(2), numSteps);

    for k = 1:numSteps
        if getOccupancy(map, [x(k), y(k)]) > 0
            isFree = false;
            return;
        end
    end
    isFree = true;
end

