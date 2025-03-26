figure()
hold on

axis([-0.1, 1, -0.1, 1])
box on

l = 1;  % length of hoop

% plotting wall corner
plot([0,1],[0,0],'k','LineWidth',1)
plot([0,0],[0,1],'k','LineWidth',1)


animation = VideoWriter('IC_falling_bar.mp4', 'MPEG-4');
animation.FrameRate = 10;
open(animation);

% draw bar
n = 100;
beta = linspace(pi/2,pi,n);

for i = 1:n
    bar = plot(l*[-cos(beta(i)),0],l*[0,sin(beta(i))],'b','LineWidth',2);

    perp_A = plot([0,1],[sin(beta(i)),sin(beta(i))],'g','Linewidth',1,'Marker','.');
    perp_B = plot([-cos(beta(i)),-cos(beta(i))],[0,1],'g','Linewidth',1,'Marker','.');
    plot_IC = plot(-cos(beta(i)),sin(beta(i)),'r','Linewidth',1,'Marker','.');

    drawnow
    writeVideo(animation, getframe(gcf))

    delete(bar)
    delete(perp_A)
    delete(perp_B)

end

close(animation)




