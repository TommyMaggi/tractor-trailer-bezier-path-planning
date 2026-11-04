function simulation_purepursuit_fixed()
    clc; close all;

    % 1) Load the rectangle path (xr, yr)
    [xr_raw, yr_raw] = path_reference();

    % Convert path points -> cumulative arc length
    sVec = [0, cumsum( sqrt(diff(xr_raw).^2 + diff(yr_raw).^2) ) ];
    totalLength = sVec(end);

    refPath.x = xr_raw;
    refPath.y = yr_raw;
    refPath.s = sVec;

    % 2) Simulation parameters (tuned a bit)
    dt        = 0.05;  % smaller dt helps reduce overshoot, also slows sim
    v         = 1.0;   % forward speed (m/s)
    L         = 1.5;   % truck wheelbase
    lookAhead = 1.5;   % smaller look‐ahead to reduce big arcs
    maxSteer  = pi/3;  % limit steering to ±60 deg so it can’t spin around
    Ltruck    = 2.0;   % rectangle length for visualization
    Wtruck    = 1.0;   
    Ltrail    = 2.0;   
    Wtrail    = 1.0;   
    hitchToTruckCenter   = Ltruck/2;
    hitchToTrailerCenter = Ltrail/2;

    % 3) Initialize truck at start
    x_t(1)     = refPath.x(1);
    y_t(1)     = refPath.y(1);
    theta_t(1) = 0;      % guess heading

    sPos(1) = 0;         % arc length “along path” we’ve covered

    % 4) Initialize trailer behind truck
    hitchX = x_t(1) - hitchToTruckCenter*cos(theta_t(1));
    hitchY = y_t(1) - hitchToTruckCenter*sin(theta_t(1));
    x_tr(1) = hitchX - hitchToTrailerCenter*cos(theta_t(1));
    y_tr(1) = hitchY - hitchToTrailerCenter*sin(theta_t(1));
    trailerTheta(1) = theta_t(1);

    figure(1); clf; set(gcf,'Name','PurePursuitFixed');

    for i = 1:10000  % big max steps, but we'll exit early
            
        % 1) If we’re near the end of one loop, stop.
        if sPos(i) >= (totalLength - 0.1)
            disp('Completed one loop around the rectangle. Stopping.');
            break;
        end

        % 2) Find look‐ahead target
        sTarget = sPos(i) + lookAhead;
        if sTarget > totalLength
            sTarget = totalLength; % we only want one loop
        end
        [xTarget, yTarget] = interpolatePath(refPath, sTarget);

        % 3) Compute steering (pure pursuit)
        xFront = x_t(i) + (L/2)*cos(theta_t(i));
        yFront = y_t(i) + (L/2)*sin(theta_t(i));
        dx = xTarget - xFront;
        dy = yTarget - yFront;
        alphaRaw = atan2(dy, dx) - theta_t(i);

        % enforce steering limit
        delta = max(-maxSteer, min(maxSteer, alphaRaw));

        % 4) Update truck (bicycle model about rear center)
        theta_dot = (v/L)*tan(delta);
        x_t(i+1) = x_t(i) + v*cos(theta_t(i))*dt;
        y_t(i+1) = y_t(i) + v*sin(theta_t(i))*dt;
        theta_t(i+1) = theta_t(i) + theta_dot*dt;

        % 5) Update arc length along path
        distMoved = v*dt;
        sPos(i+1) = sPos(i) + distMoved;

        % 6) Update trailer
        hitchX = x_t(i+1) - hitchToTruckCenter*cos(theta_t(i+1));
        hitchY = y_t(i+1) - hitchToTruckCenter*sin(theta_t(i+1));

        dxT = x_tr(i) - hitchX;
        dyT = y_tr(i) - hitchY;
        phi = atan2(dyT, dxT);

        x_tr(i+1) = hitchX + hitchToTrailerCenter*cos(phi);
        y_tr(i+1) = hitchY + hitchToTrailerCenter*sin(phi);
        trailerTheta(i+1) = phi + pi;

        % 7) Animate each iteration with a small pause
        clf; hold on; grid on; axis equal;
        title('Pure Pursuit Path Tracking (Slow & Single Loop)');
        plot(refPath.x, refPath.y, 'k-.','LineWidth',1.2);
        plot(xTarget, yTarget,'ro','MarkerSize',6,'LineWidth',2);
        plotTruckTrailer( x_t(i+1), y_t(i+1), theta_t(i+1), ...
                          x_tr(i+1), y_tr(i+1), trailerTheta(i+1), ...
                          Ltruck, Wtruck, Ltrail, Wtrail );
        axis([-8 8 -2 5]);
        drawnow;
        pause(0.07); % increase if you want it slower
    end

    % Final plots
    figure(2); clf; hold on; grid on; axis equal;
    plot(x_t, y_t, 'bo-','DisplayName','Truck center');
    plot(x_tr, y_tr, 'ro-','DisplayName','Trailer center');
    legend; title('Truck & Trailer paths'); xlabel('X'); ylabel('Y');
end

%% Interpolate path
function [xx, yy] = interpolatePath(ref, sVal)
    sData = ref.s; xData = ref.x; yData = ref.y;
    % clamp sVal to [0, sData(end)]
    if sVal <= 0, sVal = 0; end
    if sVal >= sData(end), sVal = sData(end); end

    idx = find(sData <= sVal, 1, 'last');
    if idx == length(sData)
        idx = idx-1;
    end
    s1 = sData(idx); s2 = sData(idx+1);
    t = (sVal - s1)/(s2 - s1);
    xx = xData(idx) + t*( xData(idx+1)-xData(idx) );
    yy = yData(idx) + t*( yData(idx+1)-yData(idx) );
end