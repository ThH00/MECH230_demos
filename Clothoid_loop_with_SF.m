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
N_force = m * (v.^2 ./ R - g); % Normal force at the top of the loop
N_force(N_force < 0) = 0; % Clamp negative values

% --- Animation setup ---
figure;
hold on;
plot(x_track, y_track, 'b', 'LineWidth', 2); % Plot the track
roller_coaster = plot(x_track(1), y_track(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Roller coaster

% Initialize Serret-Frenet basis vectors (T: tangent, N: normal)
scale = 5; % Scaling factor for vector visualization
T_quiver = quiver(x_track(1), y_track(1), 0, 0, 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.5); % Tangent vector (blue)
N_quiver = quiver(x_track(1), y_track(1), 0, 0, 'g', 'LineWidth', 1.5, 'MaxHeadSize', 0.5); % Normal vector (green)

title('Clothoid Roller Coaster Simulation with Serret-Frenet Basis');
xlabel('X Position');
ylabel('Y Position');
grid on;
axis equal;

% --- Calculate tangent and normal vectors ---
dx = gradient(x_track); % Derivative of x with respect to arc length
dy = gradient(y_track); % Derivative of y with respect to arc length
ds = sqrt(dx.^2 + dy.^2); % Arc length differential

% Tangent vector (T)
T = [dx ./ ds; dy ./ ds]'; % Unit tangent vector (transposed to row vectors)

% Normal vector (N = T' rotated 90° counterclockwise)
N = [-dy ./ ds; dx ./ ds]'; % Unit normal vector (perpendicular to T)

% --- Video setup ---
videoFile = 'serret_frenet_basis.mp4';
videoWriter = VideoWriter(videoFile, 'MPEG-4');
videoWriter.FrameRate = 30;
open(videoWriter);

% --- Animation loop ---
for i = 1:length(x_track)
    % Update roller coaster position
    set(roller_coaster, 'XData', x_track(i), 'YData', y_track(i));
    
    % Update tangent vector (T)
    set(T_quiver, 'XData', x_track(i), 'YData', y_track(i), ...
        'UData', T(i, 1) * scale, 'VData', T(i, 2) * scale);
    
    % Update normal vector (N)
    set(N_quiver, 'XData', x_track(i), 'YData', y_track(i), ...
        'UData', N(i, 1) * scale, 'VData', N(i, 2) * scale);
    
    % Capture the frame
    frame = getframe(gcf);
    writeVideo(videoWriter, frame);
    
    % Pause for animation effect
    pause(0.02);
end

% --- Clean up ---
close(videoWriter);
fprintf('Animation saved as %s\n', videoFile);
