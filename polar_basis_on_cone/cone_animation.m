
n = 100;
t = linspace(0,4*pi,n)';

thetadot = 1;
theta = 1*t;

er = [cos(theta), sin(theta), zeros(n,1)];
etheta = [-sin(theta), cos(theta), zeros(n,1)];

animation = VideoWriter('cone_bais.avi');
animation.FrameRate = 100;
open(animation);

ang_arr = linspace(0,5*pi,100);
b = 1;

Ez = [0, 0, 1];
r = 1*er+2*Ez;

figure()
hold on
box on
xlabel('x')
ylabel('y')
zlabel('z')
view(30,15)

plot3(r(:,1),r(:,2),r(:,3))

xlim([-2,2])
ylim([-2,2])
zlim([-1,3])

plot3(0,0,0,'*','linewidth',2,'color','k');

Ex_fixed = quiver3(0,0,0, 1, 0, 0, 'r', 'linewidth', 2);
Ey_fixed = quiver3(0,0,0, 0, 1, 0, 'g', 'linewidth', 2);
Ez_fixed = quiver3(0,0,0, 0, 0, 1, 'b', 'linewidth', 2);

text(1.1,0,0, 'Ex', 'FontSize', 10, 'Color', 'r');
text(0,1.1,0, 'Ey', 'FontSize', 10, 'Color', 'g');
text(0,0,1.1, 'Ez', 'FontSize', 10, 'Color', 'b');


for i = 1:n

    position_vector = quiver3(0,0,0,r(i,1), r(i,2), r(i,3),'k','linewidth',2);

    er_plot = quiver3(r(i,1), r(i,2), r(i,3), er(i,1), er(i,2), er(i,3), 'r', 'linewidth', 2);
    etheta_plot = quiver3(r(i,1), r(i,2), r(i,3), etheta(i,1), etheta(i,2), etheta(i,3), 'g', 'linewidth', 2);
    Ez_plot = quiver3(r(i,1), r(i,2), r(i,3), Ez(1), Ez(2), Ez(3), 'b', 'linewidth', 2);

    er_text = text(r(i,1)+er(i,1), r(i,2)+er(i,2), r(i,3)+er(i,3), 'e_r', 'FontSize', 10, 'Color', 'r');
    etheta_text = text(r(i,1)+etheta(i,1), r(i,2)+etheta(i,2), r(i,3)+etheta(i,3), 'e_{theta}', 'FontSize', 10, 'Color', 'g');
    Ez_text = text(r(i,1), r(i,2), r(i,3)+1, 'E_z', 'FontSize', 10, 'Color', 'b');

    position_vector_text = text(r(i,1)+0.1, r(i,2)+0.1, r(i,3)/2, 'r', 'FontSize', 10, 'Color', 'k');


    drawnow
    writeVideo(animation, getframe(gcf))

    pause(1)

    delete(er_plot)
    delete(etheta_plot)
    delete(Ez_plot)
    delete(position_vector)
    delete(er_text)
    delete(etheta_text)
    delete(Ez_text)
    delete(position_vector_text)

end

close(animation)