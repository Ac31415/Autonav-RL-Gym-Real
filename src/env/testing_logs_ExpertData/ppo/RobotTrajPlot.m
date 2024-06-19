% Assume robotTrajectory is your robot trajectory data of size (T, 2)
% where T is the number of time steps and the last dimension represents the x and y coordinates.

% Extract the x and y coordinates
% x = RobotYCoordinates(1:35);
% y = RobotYCoordinates(1:35);
% z = zeros(1,35)';
traj = [RobotXCoordinates(1:34) RobotYCoordinates(1:34) zeros(1,34)'];

% Plot the robot trajectory
tp = theaterPlot;
% view(14,50)
trajPlotter = trajectoryPlotter(tp,DisplayName="Trajectories");

plotTrajectory(trajPlotter,{traj})