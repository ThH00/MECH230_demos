OA = 80/1000;   % m
AB = 200/1000;  % m

n = 200;        % number of time steps

t = linspace(0,10,n);   % s, time array

wOA = 1;            % rad/s
beta = wOA*t-pi;    % rotation angle

rA = OA*[cos(beta); sin(beta)];
vA = wOA*OA*[-sin(beta); cos(beta)];

gamma = asin((0.1-OA*sin(beta))/AB);
rBwrtA = AB*[cos(gamma); sin(gamma)];
rB = rA+rBwrtA;

wAB = OA*cos(beta)/(AB*cos(gamma))*wOA;
% vB = vA+wAB*AB*[-sin(gamma);cos(gamma)];
vB = [-OA*wOA*sin(beta)-AB*wAB*sin(gamma);zeros(1,n)];

figure()
hold on
axis equal
box on

axis([-0.2, 0.4, -0.2, 0.2])

animation = VideoWriter('crank_slider.mp4', 'MPEG-4');
animation.FrameRate = 10;
open(animation);

for i = 1:n

    extension_OA = plot([-5*rA(1,i), 5*rA(1,i)],[-5*rA(2,i), 5*rA(2,i)],'g', 'Marker','.' , 'LineWidth',1);
    vB_perp = plot([rB(1,i),rB(1,i)],[-1,1],'g', 'Marker','.' , 'LineWidth',1);

    link_OA = plot([0, rA(1,i)],[0, rA(2,i)],'k','LineWidth',2);
    vect_vA = quiver(rA(1,i), rA(2,i), vA(1,i), vA(2,i),'b','LineWidth',1);
    ptA = plot(rA(1,i),rA(2,i),'r','Marker','.','LineWidth',1);

    link_AB = plot([rA(1,i), rB(1,i)],[rA(2,i),rB(2,i)],'k','LineWidth',2);
    vect_vB = quiver(rB(1,i), rB(2,i), vB(1,i), vB(2,i),'b','LineWidth',1);
    ptB = plot(rB(1,i),rB(2,i),'r','Marker','.','LineWidth',1);

    drawnow
    writeVideo(animation, getframe(gcf))
    pause(0.1)

    delete(link_OA)
    delete(vect_vA)
    delete(link_AB)
    delete(vect_vB)
    delete(extension_OA)
    delete(vB_perp)

end

close(animation)