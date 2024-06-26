% Assume lidarData is your LiDAR data of size (N, T, 2)
% where N is the number of points, T is the number of time steps,
% and the last dimension represents the x and y coordinates.
% For this example, we'll just animate the first 100 points for simplicity.

Episodes = readmatrix("Episodes.csv");
Episodes = Episodes(:, 2:end);
LiDARData = readmatrix('LiDAR_Data.csv');


ep = 1;
lid_start = 1;
lid_end = 0;

for i = 1:size(Episodes, 1)

    disp(Episodes(i));
    disp(lid_start);

    if Episodes(i) == ep

        continue

    else
        lid_end = i;

        disp(lid_end);


        vidname = "LidarDataTrainingEp"+ string(ep) + ".mp4";

        % Extract the x and y coordinates
        x = squeeze(LiDARData(lid_start:lid_end, :));
        
        % Create a new figure
        figure
        
        % set(gca,"NextPlot","replacechildren")
        set(gcf,"NextPlot","replacechildren")
        v = VideoWriter(vidname, 'MPEG-4');
        
        % decrease framerate 
        v.FrameRate = 10;              
        open(v)
        
        % Loop over the time steps
        for t = lid_start:lid_end
            % Clear the current figure
            clf
        
            % Plot the LiDAR points at the current time step
            LidData_t = LiDARData(t, :);
            angles = linspace(0,2*pi,numel(LidData_t(2:end)));
            scan = lidarScan(LidData_t(2:end),angles);
        
            plot(scan)
        
            hold on
        
            % % Define the radius of the circle
            % radius = 0.2;
            % 
            % Define the center of the circle
            centerX = 0;
            centerY = 0;
            % 
            % % Calculate the left and bottom coordinates of the circle
            % x = centerX - radius;
            % y = centerY - radius;
            % 
            % % % Create a new figure
            % % figure
            % 
            % % Plot the circle
            % rectangle('Position', [x, y, 2*radius, 2*radius], 'Curvature', [1 1], 'FaceColor', 'r')



            % Define the size of the triangle
            size = 0.5;

            % Calculate the coordinates of the vertices of the triangle
            y = [centerX - size/2, centerX + size/2, centerX];
            x = [centerY - size/2, centerY - size/2, centerY + size/2];

            % % Create a new figure
            % figure

            % Plot the triangle
            fill(x, y, 'r')


        
            hold off
        
            frame = getframe(gcf);
            % frame = getframe(gca);
            writeVideo(v,frame)
        
            % Pause for a short time to create the animation effect
            pause(0.05)
        end
        
        close(v)

        ep = ep + 1;

        lid_start = lid_end;

        % pause(1)

    end


end




% syms t
% fig = figure;
% 
% syms t
% fanimator(@fplot,cos(x)+t,sin(x)+1,[-pi pi])
% axis equal
% 
% writeAnimation(fig,'loop.gif','LoopCount',1)