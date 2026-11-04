%% path_reference.m
function [xr, yr, thetar] = path_reference()
    % A rectangular path, traveling counterclockwise:
    % bottom (x=-6→6, y=0), right (x=6, y=0→3),
    % top (x=6→-6, y=3), left (x=-6, y=3→0).
    
    N = 50;  % number of points along each side

    % bottom edge
    xb1 = linspace(-6, 6, N);
    yb1 = zeros(1,N);

    % right edge
    xr2 = 6*ones(1,N);
    yr2 = linspace(0, 3, N);

    % top edge
    xb3 = linspace(6, -6, N);
    yb3 = 3*ones(1,N);

    % left edge
    xl4 = -6*ones(1,N);
    yl4 = linspace(3, 0, N);

    % concatenate
    xr_raw = [xb1, xr2, xb3, xl4];
    yr_raw = [yb1, yr2, yb3, yl4];

    % ensure we close the loop
    xr_raw(end+1) = xr_raw(1);
    yr_raw(end+1) = yr_raw(1);

    % approximate headings by finite‐difference
    dx = diff(xr_raw);
    dy = diff(yr_raw);
    thetar = atan2(dy, dx);
    % pad last heading
    thetar(end+1) = thetar(end);

    xr = xr_raw;
    yr = yr_raw;
end