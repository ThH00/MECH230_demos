clc; clear; close all;

% Define clothoid parameters
A = 1;        % Clothoid parameter (adjustable)
s_max = 5;    % Maximum arc length
s = linspace(0, s_max, 500); % Arc length values

% Define Fresnel integrals using numerical integration
C = zeros(size(s));
S = zeros(size(s));

for i = 1:length(s)
    C(i) = integral(@(u) cos(pi * u.^2 / 2), 0, s(i));
    S(i) = integral(@(u) sin(pi * u.^2 / 2), 0, s(i));
end

% Compute clothoid coordinates
x = A * sqrt(2/pi) * C;
y = A * sqrt(2/pi) * S;

% Plot the clothoid
figure;
plot(x, y, 'b', 'LineWidth', 2);
axis equal;
grid on;
xlabel('x');
ylabel('y');
title('Clothoid (Euler Spiral)');
legend('Clothoid Curve');

% Optional: Plot curvature (κ = s/A²) along the curve
figure;
plot(s, s / A^2, 'r', 'LineWidth', 2);
grid on;
xlabel('Arc Length s');
ylabel('Curvature κ(s)');
title('Clothoid Curvature Variation');
legend('Curvature κ(s) = s/A²');
