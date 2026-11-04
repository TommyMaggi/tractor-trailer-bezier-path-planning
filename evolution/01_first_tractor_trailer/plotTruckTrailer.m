function plotTruckTrailer(xTruck, yTruck, thTruck, ...
                          xTrail, yTrail, thTrail, ...
                          Ltruck, Wtruck, Ltrail, Wtrail)

    % Truck corners in local coords
    truckCornersLocal = [ +Ltruck/2, +Wtruck/2
                          -Ltruck/2, +Wtruck/2
                          -Ltruck/2, -Wtruck/2
                          +Ltruck/2, -Wtruck/2
                          +Ltruck/2, +Wtruck/2 ];    
    
    R_truck = [ cos(thTruck), -sin(thTruck)
                sin(thTruck),  cos(thTruck) ];
    truckCornersGlobal = (R_truck * truckCornersLocal')' + [xTruck, yTruck];

    % Trailer corners in local coords
    trailerCornersLocal = [ +Ltrail/2, +Wtrail/2
                            -Ltrail/2, +Wtrail/2
                            -Ltrail/2, -Wtrail/2
                            +Ltrail/2, -Wtrail/2
                            +Ltrail/2, +Wtrail/2 ];
    R_trailer = [ cos(thTrail), -sin(thTrail)
                  sin(thTrail),  cos(thTrail) ];
    trailerCornersGlobal = (R_trailer * trailerCornersLocal')' + [xTrail, yTrail];

    % Plot the truck outline
    plot(truckCornersGlobal(:,1), truckCornersGlobal(:,2), ...
         'b-','LineWidth',1.5);

    % Plot the trailer outline
    plot(trailerCornersGlobal(:,1), trailerCornersGlobal(:,2), ...
         'b-','LineWidth',1.5);

    % Hitch line: connect truck’s rear to trailer’s front
    hitchTruck = [ xTruck - (Ltruck/2)*cos(thTruck), ...
                   yTruck - (Ltruck/2)*sin(thTruck) ];
    hitchTrail = [ xTrail + (Ltrail/2)*cos(thTrail), ...
                   yTrail + (Ltrail/2)*sin(thTrail) ];
    plot([hitchTruck(1), hitchTrail(1)], ...
         [hitchTruck(2), hitchTrail(2)], 'k--');

    % Optional: small red dot at truck center
    plot(xTruck, yTruck, 'ro','MarkerFaceColor','r','MarkerSize',5);
end