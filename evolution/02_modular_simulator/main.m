%% Main simulation
%[x_ref, y_ref] = generate_path();
%[truck, trailer] = simulate_vehicle(x_ref, y_ref);

%% Run animation in main plot
%animate_vehicles(x_ref, y_ref, truck, trailer);

%% Generate post-simulation plots after animation completes
%plot_results(truck, trailer);

% Main simulation script
[x_ref, y_ref] = generate_path();
[truck, trailer] = simulate_vehicle(x_ref, y_ref);
animate_vehicles(x_ref, y_ref, truck, trailer);
plot_results(truck, trailer);