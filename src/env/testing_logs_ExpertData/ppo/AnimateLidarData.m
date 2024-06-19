% Assume lidarData is your LiDAR data of size (N, T, 2)
% where N is the number of points, T is the number of time steps,
% and the last dimension represents the x and y coordinates.
% For this example, we'll just animate the first 100 points for simplicity.

% Extract the x and y coordinates
x = squeeze(LiDARData(1:34, :));

% Create a new figure
figure

% Loop over the time steps
for t = 1:size(x, 1)
    % Clear the current figure
    clf

    % Plot the LiDAR points at the current time step
    LidData_t = LiDARData(t, :);
    angles = linspace(0,2*pi,numel(LidData_t(2:end)));
    scan = lidarScan(LidData_t(2:end),angles);

    plot(scan)

    % Pause for a short time to create the animation effect
    pause(0.10)
end