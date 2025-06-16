OA = 80/1000; % m
AB = 200/1000; % m
n = 200; % number of time steps
t = linspace(0,10,n); % s, time array
wOA = 1; % rad/s
beta = wOA*t-pi; % rotation angle

% Position and velocity of point A
rA = OA*[cos(beta); sin(beta)];
vA = wOA*OA*[-sin(beta); cos(beta)];

% Angle gamma and position of point B
gamma = asin((0.1-OA*sin(beta))/AB);
rBwrtA = AB*[cos(gamma); sin(gamma)];
rB = rA+rBwrtA;

% Angular velocity of link AB
wAB = OA*cos(beta)./(AB*cos(gamma))*wOA;

% Velocity of point B
vB = [-OA*wOA*sin(beta)-AB*wAB.*sin(gamma); zeros(1,n)];

% Calculate Instantaneous Center (IC) coordinates
% IC is at the intersection of:
% 1) Extension of OA (line through O and A)
% 2) Perpendicular to slider motion through B
% Since slider moves horizontally, IC has same x-coordinate as B and lies on line OA

% IC lies on the line from O through A, so IC = k * rA for some scalar k
% IC also has x-coordinate equal to rB(1,:) (same x as point B)
% Therefore: k * rA(1,:) = rB(1,:), so k = rB(1,:) ./ rA(1,:)
k = rB(1,:) ./ rA(1,:); % scaling factor
rIC = [rB(1,:); k .* rA(2,:)]; % IC position

% Store IC trajectory for plotting
IC_trajectory_x = rIC(1,:);
IC_trajectory_y = rIC(2,:);

% Create figure and animation
figure()
set(gcf, 'Color', 'white'); % White background
hold on
axis equal
box on
axis([-0.2, 0.4, -0.4, 0.4]) % Extended y-axis to show IC movement
title('Crank-Slider Mechanism with Instantaneous Center')

animation = VideoWriter('crank_slider_with_IC.mp4', 'MPEG-4');
animation.FrameRate = 10;
open(animation);

for i = 1:n
    % Slider track (fixed rail)
    slider_track = plot([-0.1, 0.4], [0.1, 0.1], 'k-', 'LineWidth', 4);
    slider_track_bottom = plot([-0.1, 0.4], [0.09, 0.09], 'k-', 'LineWidth', 2);
    
    % Slider block dimensions
    block_width = 0.03;
    block_height = 0.04;
    block_x = rB(1,i) - block_width/2;
    block_y = 0.1 - block_height/2;
    
    % Slider block
    slider_block = rectangle('Position', [block_x, block_y, block_width, block_height], ...
                            'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'k', 'LineWidth', 2);
    
    % Extension lines
    extension_OA = plot([-5*rA(1,i), 5*rA(1,i)], [-5*rA(2,i), 5*rA(2,i)], 'g:', 'LineWidth', 1);
    vB_perp = plot([rB(1,i), rB(1,i)], [-1, 1], 'g:', 'LineWidth', 1);
    
    % Mechanism links
    link_OA = plot([0, rA(1,i)], [0, rA(2,i)], 'k', 'LineWidth', 3);
    link_AB = plot([rA(1,i), rB(1,i)], [rA(2,i), rB(2,i)], 'k', 'LineWidth', 3);
    
    % Velocity vectors
    vect_vA = quiver(rA(1,i), rA(2,i), vA(1,i), vA(2,i), 'b', 'LineWidth', 2, 'MaxHeadSize', 0.8);
    vect_vB = quiver(rB(1,i), rB(2,i), vB(1,i), vB(2,i), 'b', 'LineWidth', 2, 'MaxHeadSize', 0.8);
    
    % Joint points
    ptO = plot(0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
    ptA = plot(rA(1,i), rA(2,i), 'ro', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
    ptB = plot(rB(1,i), rB(2,i), 'ro', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
    
    % Instantaneous Center
    ptIC = plot(rIC(1,i), rIC(2,i), 'mo', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
    
    % Lines from IC to points A and B to show zero relative velocity
    line_IC_A = plot([rIC(1,i), rA(1,i)], [rIC(2,i), rA(2,i)], 'm--', 'LineWidth', 1);
    line_IC_B = plot([rIC(1,i), rB(1,i)], [rIC(2,i), rB(2,i)], 'm--', 'LineWidth', 1);
    
    drawnow
    writeVideo(animation, getframe(gcf))

    
    % Clean up dynamic elements (keep IC trajectory)
    delete(slider_track)
    delete(slider_track_bottom)
    delete(slider_block)
    delete(extension_OA)
    delete(vB_perp)
    delete(link_OA)
    delete(link_AB)
    delete(vect_vA)
    delete(vect_vB)
    delete(ptO)
    delete(ptA)
    delete(ptB)
    delete(line_IC_A)
    delete(line_IC_B)
end

close(animation);

% Plot final configuration with IC information
figure()
set(gcf, 'Color', 'white'); % White background
hold on
axis equal
box on
grid on
title('Instantaneous Center Analysis')

% Plot mechanism at several positions
positions = [1, 50, 100, 150, 200];
colors = ['r', 'g', 'b', 'c', 'k'];

for j = 1:length(positions)
    i = positions(j);
    plot([0, rA(1,i)], [0, rA(2,i)], colors(j), 'LineWidth', 2);
    plot([rA(1,i), rB(1,i)], [rA(2,i), rB(2,i)], colors(j), 'LineWidth', 2);
    plot(rIC(1,i), rIC(2,i), [colors(j) 'o'], 'MarkerFaceColor', colors(j), 'MarkerSize', 8);
end


% Display some analysis
fprintf('Instantaneous Center Analysis:\n');
fprintf('Max distance from origin: %.3f m\n', max(sqrt(rIC(1,:).^2 + rIC(2,:).^2)));
fprintf('Min distance from origin: %.3f m\n', min(sqrt(rIC(1,:).^2 + rIC(2,:).^2)));
fprintf('Y-coordinate range: %.3f to %.3f m\n', min(rIC(2,:)), max(rIC(2,:)));