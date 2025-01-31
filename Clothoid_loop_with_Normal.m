clc;
clear;
close all;

% Parameters
c = 0.00375; % Constant of proportionality
s_max = 41.3; % Maximum parameter limit to allow full clothoid loop
t_values = linspace(0, s_max, 300); % Parameter range
g = 9.81; % Gravitational acceleration (m/s^2)
m = 100; % Mass of the rollercoaster (kg)
ho = 50; % Initial height of the drop (m)


% Define clothoid integral functions based on the given equation
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

% Mirror the shape about x = 15.3557
x_mirrored = 2 * 15.3557 - x_values;

% Combine original and mirrored tracks correctly
x_track = [x_values, flip(x_mirrored)];
y_track = [y_values, flip(y_values)];

% calculating the tan vector
dx = zeros(length(x_track),1);
dy = zeros(length(y_track),1);
for i = 2:length(x_track)
    dx(i) = x_track(i) - x_track(i-1);
    dy(i) = y_track(i) - y_track(i-1);
end
ds = sqrt(dx.^2 + dy.^2);
et = [dx,dy,zeros(length(x_track),1)]./ds;
Ez = [0,0,1];

figure(100)
hold on
plot(x_track,y_track)
for i=1:10:length(x_track)
    en = cross(Ez,et(i,:));
    quiver(x_track(i),y_track(i),3*en(1),3*en(2),'r');

    quiver(x_track(i),y_track(i),10*et(i,1),10*et(i,2),'b');
end

% Compute velocity based on conservation of energy
h_initial = max(y_track);
v = sqrt(2 * g * (h_initial - y_track));

% Compute time for each segment
dt = 0.05; % Time step
time = [0 cumsum(sqrt(diff(x_track).^2 + diff(y_track).^2) ./ v(1:end-1))];
time = [time, time(end)];

% Animation setup
figure;
hold on;
plot(x_track, y_track, 'b', 'LineWidth', 2);
roller_coaster = plot(x_track(1), y_track(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
title('Clothoid Roller Coaster Simulation');
xlabel('X Position');
ylabel('Y Position');
grid on;
axis equal;

% Animation loop
for i = 1:length(x_track)
    set(roller_coaster, 'XData', x_track(i), 'YData', y_track(i));
    pause(0.02);
end


