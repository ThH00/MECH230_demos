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
X_clothoid = @(s) arrayfun(@(u) integral(@(z) cos((c * z.^2)/2), 0, u, 'ArrayValued', true), s);
Y_clothoid = @(s) arrayfun(@(u) integral(@(z) sin((c * z.^2)/2), 0, u, 'ArrayValued', true), s);

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

% Velocity calculation
h_initial = ho; % Use the initial height
v = sqrt(2 * g * (h_initial - y_track));

% Normal force calculation
R = 1 ./ k_track; % Radius of curvature
N = m * (v.^2 ./ R - g); % Normal force at the top of the loop

% Clamp normal force to zero if negative
N(N < 0) = 0;

% --- Animation setup ---
figure;
hold on;
plot(x_track, y_track, 'b', 'LineWidth', 2); % Plot the track
roller_coaster = plot(x_track(1), y_track(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Roller coaster

% Initialize polar basis vectors (e_r and e_theta)
origin_x = x_track(1); % Origin is the start of the track
origin_y = y_track(1);
scale = 5; % Scaling factor for vector visualization

% Radial vector (e_r) in blue
e_r_quiver = quiver(origin_x, origin_y, 0, 0, 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

% Tangential vector (e_theta) in green
e_theta_quiver = quiver(origin_x, origin_y, 0, 0, 'g', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

title('Clothoid Roller Coaster Simulation with Polar Basis');
xlabel('X Position');
ylabel('Y Position');
grid on;
axis equal;

% --- Video setup ---
videoFile = 'clothoid_polar_basis.mp4';
videoWriter = VideoWriter(videoFile, 'MPEG-4');
videoWriter.FrameRate = 30;
open(videoWriter);

% --- Animation loop ---
for i = 1:length(x_track)
    % Current position of the roller coaster
    current_x = x_track(i);
    current_y = y_track(i);
    
    % Update roller coaster position
    set(roller_coaster, 'XData', current_x, 'YData', current_y);
    
    % Calculate polar basis vectors relative to the origin (track start)
    dx = current_x - origin_x;
    dy = current_y - origin_y;
    magnitude = sqrt(dx^2 + dy^2);
    
    if magnitude > 0
        % Radial unit vector (e_r)
        e_r_x = dx / magnitude;
        e_r_y = dy / magnitude;
        
        % Tangential unit vector (e_theta = Ez × e_r)
        e_theta_x = -e_r_y; % Rotate e_r 90 degrees counterclockwise
        e_theta_y = e_r_x;
    else
        % At origin, set vectors to zero
        e_r_x = 0;
        e_r_y = 0;
        e_theta_x = 0;
        e_theta_y = 0;
    end
    
    % Update quiver vectors (scaled for visualization)
    set(e_r_quiver, 'XData', current_x, 'YData', current_y, ...
        'UData', e_r_x * scale, 'VData', e_r_y * scale);
    set(e_theta_quiver, 'XData', current_x, 'YData', current_y, ...
        'UData', e_theta_x * scale, 'VData', e_theta_y * scale);
    
    % Capture the frame
    frame = getframe(gcf);
    writeVideo(videoWriter, frame);
    
    % Pause for animation effect
    pause(0.02);
end

% --- Clean up ---
close(videoWriter);
fprintf('Animation saved as %s\n', videoFile);