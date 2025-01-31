clc;
clear;
close all;

% Parameters
g = 9.81; % Gravitational acceleration (m/s^2)
r = 20; % Radius of the loop (m)
ho = 50; % Initial height of the drop (m)
vo = sqrt(2 * g * ho); % Initial velocity at the bottom (m/s)

flat_length = 30; % Length of flat track before and after the loop
theta_loop = linspace(0, 2*pi, 200); % Loop angles in radians
theta_flat_before = linspace(-pi/2, 0, 50); % Flat track before loop mapped to angles
theta_flat_after = linspace(2*pi, 2.5*pi, 50); % Flat track after loop mapped to angles

% Geometry of the flat track, loop, and exit
x_flat_before = linspace(0, flat_length, 50); % Flat track x-coordinates
h_flat_before = zeros(size(x_flat_before)); % Flat track height (y = 0)

x_loop = flat_length + r * sin(theta_loop); % Loop x-coordinates
h_loop = r * (1 - cos(theta_loop)); % Heights of the loop

x_flat_after = linspace(x_loop(end), x_loop(end) + flat_length, 50); % Exit flat track x-coordinates
h_flat_after = zeros(size(x_flat_after)); % Flat track height (y = 0)

% Combine all segments
x_track = [x_flat_before, x_loop, x_flat_after];
h_track = [h_flat_before, h_loop, h_flat_after];
theta = [theta_flat_before, theta_loop, theta_flat_after]; % Full angular range

% Velocity calculation
h = [h_flat_before, h_loop, h_flat_after]; % Combined height profile
v = sqrt(2 * g * (ho - h)); % Velocity as a function of height
v_change = diff(v); % Change in velocity at each step

% G-force calculation
G_flat = ones(size(h_flat_before)); % G-force on flat track (1g)
G_loop = (v(length(h_flat_before)+1:length(h_flat_before)+length(h_loop)).^2) / (r * g) + cos(theta_loop); % G-force in loop
G_after = ones(size(h_flat_after)); % G-force on exit flat track (1g)
G = [G_flat, G_loop, G_after]; % Combine G-forces

% Time calculation
dt = 0.05; % Time step for simulation (s)
time = [0 cumsum(sqrt(diff(x_track).^2 + diff(h_track).^2) ./ v(1:end-1))]; % Total time at each point

% Animation setup
figure;
hold on;
plot(x_track, h_track, 'k', 'LineWidth', 2); % Draw track
roller_coaster = plot(x_track(1), h_track(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Roller coaster point
title('Realistic Roller Coaster Simulation with Flat Sections');
xlabel('Horizontal Position (m)');
ylabel('Height (m)');
axis equal;
grid on;

% Animation loop
for i = 1:length(x_track)
    % Update roller coaster position
    set(roller_coaster, 'XData', x_track(i), 'YData', h_track(i));
    
    % Display velocity, g-force, and velocity change
    if i > 1
        fprintf('Position: %.2f m | Height: %.2f m | Velocity: %.2f m/s | G-Force: %.2f | ΔVelocity: %.2f m/s\n', ...
            x_track(i), h_track(i), v(i), G(i), v_change(i-1));
    else
        fprintf('Position: %.2f m | Height: %.2f m | Velocity: %.2f m/s | G-Force: %.2f\n', ...
            x_track(i), h_track(i), v(i), G(i));
    end
    
    % Pause for animation effect
    pause(0.02);
end

% Plot G-force vs loop angle (including flat sections)
figure;
plot(theta, G, 'b', 'LineWidth', 1.5);
title('G-Force vs Loop Angle (Including Flat Sections)');
xlabel('Angle (radians)');
ylabel('G-Force (G)');
grid on;

% Plot duration of each G-force level
G_bins = linspace(min(G), max(G), 20); % Define G-force bins
G_durations = histcounts(G, G_bins) * dt; % Calculate duration for each bin
bin_centers = (G_bins(1:end-1) + G_bins(2:end)) / 2;

figure;
bar(bin_centers, G_durations, 'b');
title('Duration of Each G-Force Level');
xlabel('G-Force (G)');
ylabel('Duration (s)');
grid on;
