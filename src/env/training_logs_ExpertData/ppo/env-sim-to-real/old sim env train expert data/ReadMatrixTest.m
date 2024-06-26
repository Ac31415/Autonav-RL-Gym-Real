LiDARs = readmatrix('LiDAR_Data.csv');
goal_xs = readmatrix('Goal_X_Coordinates.csv');
goal_ys = readmatrix('Goal_Y_Coordinates.csv');
rob_xs = readmatrix('Robot_X_Coordinates.csv');
rob_ys = readmatrix('Robot_Y_Coordinates.csv');
episodes = readmatrix("Episodes.csv");
% % Remove the first column of the matrix
% episodes = episodes(:, 2:end);
