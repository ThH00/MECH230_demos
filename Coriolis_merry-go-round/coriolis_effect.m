g = 9.81;      % gravitational acceleration
R = 1;          % radius of the merry-go-round
r0 = [R;0;0];   % initial position vector of ball, ball being thrown from edge of merry-go-round
v0 = [-1;0;0];  % initial velocity of ball

% defining my time array
n = 100;
t = linspace(0,10,n);

a = [0;0;-g];   % acceleration of the ball
v = zeros(3,n);
r = zeros(3,n);
for i = 1:n
    v(:,i) = a*t(i)+v0;
    r(:,i) = a*t(i)^2/2+v0*t(i)+r0;
end




omega = 0.1;     % angular velocity
theta = omega*t;    % rotation angle as a function of time
ex = [cos(theta); sin(theta); zeros(1,n)];
ey = [-sin(theta); cos(theta); zeros(1,n)];

xC = zeros(n,1);
yC = zeros(n,1);

for i = 1:n
    xC(i) = dot(r(:,i),ex(:,i));
    yC(i) = dot(r(:,i),ey(:,i));
end

figure()
hold on
plot3(r(1,:),r(2,:),r(3,:))
xlabel('x')
ylabel('y')
zlabel('z')
plot(xC,yC)

animation = VideoWriter('coriolis', 'MPEG-4');
animation.FrameRate = 10;
open(animation);


figure()

hold on

for i = 1:n
    subplot(1,2,1)
    axis([-1,1,-1,1])
    plot(r(1,i),r(2,i),'*','Color','k','linewidth',1)
    ex_quiver = quiver(0,0,ex(1,i),ex(2,i),'Color','b','linewidth',1);
    axis([-1,1,-1,1])

    subplot(1,2,2)
    axis([-1,1,-1,1])
    plot(xC(i),yC(i),'*','Color','b','linewidth',1)
    axis([-1,1,-1,1])

    drawnow
    writeVideo(animation, getframe(gcf))

    delete(ex_quiver)

end
close(animation)
