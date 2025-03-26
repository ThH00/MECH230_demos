clc;
clear;
close all;

% Define clothoid parameters
a = 1;        % Clothoid parameter (adjustable)
s_max = 5;    % Maximum arc length
n = 500;        % nimber of values
s = linspace(0, s_max, n); % Arc length values

% Define Fresnel integrals using numerical integration
C = zeros(size(s));
S = zeros(size(s));

for i = 1:length(s)
    C(i) = integral(@(u) cos(pi * u.^2 / 2), 0, s(i));
    S(i) = integral(@(u) sin(pi * u.^2 / 2), 0, s(i));
end

% Compute clothoid coordinates
x = a * sqrt(2/pi) * C;
y = a * sqrt(2/pi) * S;

% Cutting the clothoid at its max value
[max_val,n] = max(y);
x = x(1:n);
y = y(1:n);
s = s(1:n);

% Adding the symmetric part to the clothoid
x = [x, 2*x(n)-flip(x)];
y = [y, flip(y)];
s = [s, flip(s)];

% Optional: Plot curvature (κ = s/A²) along the curve
figure;
kappa = s / a^2;
plot(s, kappa, 'r', 'LineWidth', 2);
grid on;
xlabel('Arc Length s');
ylabel('Curvature κ(s)');
title('Clothoid Curvature Variation');
legend('Curvature κ(s) = s/A²');

% Plot the clothoid
figure;
hold on
plot(x, y, 'k', 'LineWidth', 2);
axis equal;
grid on;
xlabel('x');
ylabel('y');
title('Clothoid (Euler Spiral)');

% Plotting the unit tangent and the unit normal
% Expressesions obtained from OOR Primer figure 3.3
for i = 1:n
    et = [cos(s.^2./(2*a.^2)); sin(s.^2./(2*a.*2))];
    en = [-sin(s.^2./(2*a.*2)); cos(s.^2./(2*a.^2))];
end
for i = n:2*n
    et = -[cos(s.^2./(2*a.^2)); sin(s.^2./(2*a.*2))];
    en = [sin(s.^2./(2*a.*2)); cos(s.^2./(2*a.^2))];
end
% plotting the scaled unit vectors
for i = 1:20:2*n
    quiver(x(i),y(i),0.1*et(1,i),0.1*et(2,i),'b','LineWidth',2);
    quiver(x(i),y(i),0.1*en(1,i),0.1*en(2,i),'r','LineWidth',2);
end

% Calculating the normal force
g = 9.81;   % gravitational acceleration
m = 1;
y0 = max(y);
v_squared = 2*g*(y0-y);
N = m*(kappa.*v_squared+g*cos(s.^2./(2*a.^2)));

figure()
plot(N)