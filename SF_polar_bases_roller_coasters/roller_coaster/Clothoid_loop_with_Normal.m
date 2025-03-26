clc;
clear;
close all;

% Parameters
c = 0.00375; % Constant of proportionality
s_max = 41.3; % Maximum parameter limit to allow full clothoid loop
t_values = linspace(0, s_max, 300); % Parameter range
g = 9.81; % Gravitational acceleration (m/s^2)
m = 50; % Mass of the rollercoaster (kg)
ho = 50; % Initial height of the drop (m)

% Define clothoid integral functions
X_clothoid = @(s) arrayfun(@(u) integral(@(z) cos((c * z.^2) / 2), 0, u, 'ArrayValued', true), s);
Y_clothoid = @(s) arrayfun(@(u) integral(@(z) sin((c * z.^2) / 2), 0, u, 'ArrayValued', true), s);

% Compute clothoid coordinates
x_values = X_clothoid(t_values);
y_values = Y_clothoid(t_values);

% Find the highest point (where y reaches max)
[~, idx_max] = max(y_values);

% Trim the loop to stop at its highest point
x_values = x_values(1:idx_max);
y_values = y_values(1:idx_max);

% Mirror the shape about x = 15.355
x_mirrored = 2 * 15.3557 - x_values;

% Combine original and mirrored tracks
x_track = [x_values, flip(x_mirrored)];
y_track = [y_values, flip(y_values)];

% Curvature calculation
k_values = c * t_values(1:idx_max);
k_mirrored = flip(k_values);
k_track = [k_values, k_mirrored];

<<<<<<< HEAD:Clothoid_loop_with_Normal.m
% Velocity calculation
h_initial = ho; % Use the initial height
v = sqrt(2 * g * (h_initial - y_track));

% Normal force calculation
R = 1 ./ k_track; % Radius of curvature
N = m * (v.^2 ./ R - g); % Normal force at the top of the loop

% Clamp normal force to zero if negative
N(N < 0) = 0;
=======
figure(100)
hold on
plot(x_track,y_track)
en = zeros(length(x_track),3);
for i=1:10:length(x_track)
    en(i,:) = cross(Ez,et(i,:));
    quiver(x_track(i),y_track(i),3*en(i,1),3*en(i,2),'r','MaxHeadSize',0.5);

    quiver(x_track(i),y_track(i),10*et(i,1),10*et(i,2),'b');
end

% Compute velocity based on conservation of energy
h_initial = max(y_track);
v = sqrt(2 * g * (h_initial - y_track));
N = zeros(length(x_track));
%compute normal force
for i = 1:length(x_track)
    N(i) = m * (k_track(i) .* (v(i).^2) + g * dot([0,1,0],en(i,:)));
end

animation = VideoWriter('clothoid_rollercoaster.avi');
animation.FrameRate = 100;
open(animation);
>>>>>>> master:roller_coaster/Clothoid_loop_with_Normal.m

% --- Animation setup ---
figure;
hold on;
plot(x_track, y_track, 'b', 'LineWidth', 2); % Plot the track
roller_coaster = plot(x_track(1), y_track(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Roller coaster
title('Clothoid Roller Coaster Simulation');
xlabel('X Position');
ylabel('Y Position');
grid on;
axis equal;

<<<<<<< HEAD:Clothoid_loop_with_Normal.m
% --- Calculate tangent and normal vectors ---
dx = gradient(x_track); % Derivative of x with respect to arc length (column vector)
dy = gradient(y_track); % Derivative of y with respect to arc length (column vector)
ds = sqrt(dx.^2 + dy.^2); % Arc length differential (column vector)

% Ensure dx, dy, and ds are column vectors
dx = dx(:);
dy = dy(:);
ds = ds(:);

% Tangent vector (column vectors)
et = [dx ./ ds, dy ./ ds, zeros(length(x_track), 1)];

% Z-axis unit vector
Ez = [0, 0, 1];

% Normal vector (perpendicular to tangent)
en = cross(repmat(Ez, length(x_track), 1), et, 2); % Cross product along rows (flipped order)

% Initialize quiver object
quiver_handle = quiver(x_track(1), y_track(1), en(1, 1) * N(1)/1000, en(1, 2) * N(1)/1000, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

% --- Video setup ---
videoFile = 'clothoid_roller_coaster.mp4'; % Output video file name
videoWriter = VideoWriter(videoFile, 'MPEG-4'); % Create VideoWriter object
videoWriter.FrameRate = 30; % Set frame rate (frames per second)
open(videoWriter); % Open the video file for writing

% --- Animation loop ---
=======
quiver_arrow = quiver(x_track(1),y_track(1),N(1)*en(i,1),N(1)*en(i,2),'r','MaxHeadSize',0.5);

% --- Animation loop with display of N ---
>>>>>>> master:roller_coaster/Clothoid_loop_with_Normal.m
for i = 1:length(x_track)
    % Update roller coaster position
    set(roller_coaster, 'XData', x_track(i), 'YData', y_track(i));
    set(quiver_arrow, 'XData', x_track(i), 'YData', y_track(i), 'UData', N(i)*en(i,1), 'VData', N(i)*en(i,2));

    drawnow
    writeVideo(animation, getframe(gcf))
    
    % Update quiver arrow (normal force direction)
    if N(i) > 0
        set(quiver_handle, 'XData', x_track(i), 'YData', y_track(i), ...
            'UData', en(i, 1) * N(i)/1000, 'VData', en(i, 2) * N(i)/1000);
    else
        % Hide quiver if normal force is zero
        set(quiver_handle, 'UData', 0, 'VData', 0);
    end
    
    % Update title with normal force value
    title(sprintf('Normal Force: %.2f N', N(i)));
    
    % Capture the current frame
    frame = getframe(gcf); % Capture the entire figure
    writeVideo(videoWriter, frame); % Write the frame to the video
    
    % Pause for animation effect
    pause(0.02);
end

<<<<<<< HEAD:Clothoid_loop_with_Normal.m
% --- Clean up ---
close(videoWriter); % Close the video file
fprintf('Animation saved as %s\n', videoFile);
=======
close(animation)

>>>>>>> master:roller_coaster/Clothoid_loop_with_Normal.m
