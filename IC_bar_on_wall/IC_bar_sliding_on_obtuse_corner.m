figure()
hold on

axis([-0.1, 1.1, -0.6, 1.1])
% axis off;
box on

l = 1;  % length of hoop

% plotting wall corner
gamma = pi/6;
plot([0,cos(gamma)],[0,-sin(gamma)],'k','LineWidth',1)
plot([0,0],[0,1],'k','LineWidth',1)


animation = VideoWriter('IC_falling_bar_obtuse.mp4', 'MPEG-4');
animation.FrameRate = 10;
open(animation);

% draw bar
n = 100;
OB = linspace(0,l,n);
beta = asin(OB*sin(pi/2+gamma)/l);
delta = pi/2-gamma-beta;
OA = l*sin(delta)/sin(pi/2+gamma);

Ex = [1,0];
Ey = [0,1];
u = cos(gamma)*Ex-sin(gamma)*Ey;
v = cos(gamma)*Ey+sin(gamma)*Ex;

for i = 1:n

    axis equal
    axis([-0.1, 1.1, -0.6, 1.1])

    rA = OA(i)*Ey;
    rB = OB(i)*u;

    rIC = rA+l*sin(pi/2-delta(i))/sin(beta(i)+delta(i))*Ex;

    bar = plot([rA(1), rB(1)], [rA(2), rB(2)],'b','LineWidth',2);

    perp_A = plot([0,2],[rA(2),rA(2)],'g','Linewidth',1,'Marker','.');
    perp_B = plot([rB(1),rB(1)+2*v(1)],[rB(2),rB(2)+2*v(2)],'g','Linewidth',1,'Marker','.');
    plot_IC = plot(rIC(1),rIC(2),'r','Linewidth',1,'Marker','.');

    drawnow
    writeVideo(animation, getframe(gcf))

    delete(bar)
    delete(perp_A)
    delete(perp_B)

end

close(animation)




