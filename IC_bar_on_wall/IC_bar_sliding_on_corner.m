figure()
set(gcf, 'Color', 'w');  % Set white background for the figure
hold on
axis([-0.1, 1.1, -0.1, 1.1])
axis off;
box on



l = 1;  % length of the bar

% Draw fixed walls
plot([0, 1], [0, 0], 'k', 'LineWidth', 1)
plot([0, 0], [0, 1], 'k', 'LineWidth', 1)

% Set up video writer
animation = VideoWriter('IC_falling_bar.mp4', 'MPEG-4');
animation.FrameRate = 10;
open(animation);

% Animation loop
n = 100;
beta = linspace(pi/2, pi, n);

for i = 1:n
    % Draw bar
    bar = plot(l * [-cos(beta(i)), 0], l * [0, sin(beta(i))], 'b', 'LineWidth', 2);

    % Perpendiculars and IC
    perp_A = plot([0, 1], [sin(beta(i)), sin(beta(i))], 'g', 'LineWidth', 1, 'Marker', '.');
    perp_B = plot([-cos(beta(i)), -cos(beta(i))], [0, 1], 'g', 'LineWidth', 1, 'Marker', '.');
    plot_IC = plot(-cos(beta(i)), sin(beta(i)), 'ro', 'MarkerFaceColor', 'r');

    % Capture frame
    drawnow
    frame = getframe(gcf);
    writeVideo(animation, frame);

    % Remove current frame's elements
    delete(bar)
    delete(perp_A)
    delete(perp_B)
    delete(plot_IC)
end

% Finish recording
close(animation);
