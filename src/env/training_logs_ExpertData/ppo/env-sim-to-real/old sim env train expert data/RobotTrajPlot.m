% Assume robotTrajectory is your robot trajectory data of size (T, 2)
% where T is the number of time steps and the last dimension represents the x and y coordinates.

% Extract the x and y coordinates
% x = RobotYCoordinates(1:35);
% y = RobotYCoordinates(1:35);
% z = zeros(1,35)';


Episodes = readmatrix("Episodes.csv");
Episodes = Episodes(:, 2:end);

RobotXCoordinates = readmatrix("Robot_X_Coordinates.csv");
RobotXCoordinates = RobotXCoordinates(:, 2:end);

RobotYCoordinates = readmatrix("Robot_Y_Coordinates.csv");
RobotYCoordinates = RobotYCoordinates(:, 2:end);

GoalXCoordinates = readmatrix("Goal_X_Coordinates.csv");
GoalXCoordinates = GoalXCoordinates(:, 2:end);

GoalYCoordinates = readmatrix("Goal_Y_Coordinates.csv");
GoalYCoordinates = GoalYCoordinates(:, 2:end);





ep = 1;
traj_start = 1;
traj_end = 0;



for i = 1:size(Episodes, 1)

    disp(Episodes(i));
    disp(traj_start);

    if Episodes(i) == ep

        continue

    else
        traj_end = i;

        disp(traj_end);


        traj = [RobotXCoordinates(traj_start:traj_end-1) RobotYCoordinates(traj_start:traj_end-1) zeros(1, size(RobotYCoordinates(traj_start:traj_end-1), 1))'];
        % traj = [RobotXCoordinates(35:81) RobotYCoordinates(35:81) zeros(1, size(RobotYCoordinates(35:81), 1))'];
        % traj = [RobotXCoordinates(82:99) RobotYCoordinates(82:99) zeros(1, size(RobotYCoordinates(82:99), 1))'];
        % traj = [RobotXCoordinates(100:125) RobotYCoordinates(100:125) zeros(1, size(RobotYCoordinates(100:125), 1))'];
        
        % Plot the robot trajectory
        % tp = theaterPlot("XLimits", [-5, 5], "YLimits", [-5, 5]);
        tp = theaterPlot;
        % view(14,50)
        trajPlotter = trajectoryPlotter(tp,DisplayName="Trajectory", LineWidth=3);
        
        plotTrajectory(trajPlotter,{traj})
        
        hold on
        
        
        
        % Define the width and height of the rectangle
        width = 0.25;
        height = 0.25;
        
        % Define the center of the rectangle
        centerX = GoalXCoordinates(traj_start);
        centerY = GoalYCoordinates(traj_start);
        
        % Calculate the bottom left corner of the rectangle
        x = centerX - width / 2;
        y = centerY - height / 2;
        
        % Plot the rectangle
        rectangle('Position', [x, y, width, height], 'FaceColor', 'r')
        
        
        
        plot(GoalXCoordinates(traj_start), GoalYCoordinates(traj_start), 'ro')
        % rect = rectangle('Position', [GoalXCoordinates(1), GoalYCoordinates(1), 0.5, 0.5], 'FaceColor', 'r');
        % rect = rectangle('Position', [GoalXCoordinates(1), GoalYCoordinates(1), 0.5, 0.5]);
        % plot(GoalXCoordinates(35), GoalYCoordinates(35), 'ro')
        % plot(GoalXCoordinates(82), GoalYCoordinates(82), 'ro')
        
        % Plot the square
        % rectangle('Position', [GoalXCoordinates(100), GoalYCoordinates(100), 0.5, 0.5], 'EdgeColor', 'b')
        
        % % Add a legend
        % legend(rect, 'My Square')
        
        % t = annotation("textbox");
        % t.String = 'Goal';
        % t.Position = [x y 0 0];
        % % t.Position = [1, 1, 0.1, 0.1];
        
        % Plot the text
        text(GoalXCoordinates(traj_start), GoalYCoordinates(traj_start), 'Goal', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle')



        % Define the radius of the circle
        radius = 0.2;

        % Define the center of the circle
        centerX = 0;
        centerY = 0;

        % Calculate the left and bottom coordinates of the circle
        x = centerX - radius;
        y = centerY - radius;

        % % Create a new figure
        % figure

        % Plot the circle
        rectangle('Position', [x, y, 2*radius, 2*radius], 'Curvature', [1 1], 'FaceColor', 'g')

        % Plot the text
        text(centerX, centerY, 'Start', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle')


        
        hold off

        plotname = "TrajDataTrainingEp"+ string(ep) + ".png";

        % Save the plot as an image
        saveas(gcf, plotname);

        ep = ep + 1;

        traj_start = traj_end;

        pause(1)

    end


end



