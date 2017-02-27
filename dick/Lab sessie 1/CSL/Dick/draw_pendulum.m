%--------------Dynamic Plotting--------------
% Video of the double pendulum
% Create the markers representing the pendulum which will 
% vary according to calculated positions
figure(1)
h=plot(0,0,'MarkerSize',20,'Marker','+','LineWidth',5); %the BC
hold on
h1=plot(0,0,'MarkerSize',30,'Marker','.','LineWidth',2,'Color','g'); %the masses
hold off
%configure the axis to slightly larger than the lines for a suitable
%animation space
range=1.1*(l1+l2); axis([-range range -range range]); axis square;
title('Pendulum Animation')
xlabel('x displacement'); ylabel('y displacement');
set(gca,'nextplot','replacechildren');  %line to fresh each plot

for i=1:length(theta1)-1
    if (ishandle(h1)==1) %check figure is plotting
        %x cordinate of mass 1 and 2 at current time step
        Xcoord=[0,l1*sin(theta1(i)),l1*sin(theta1(i))+l2*sin(theta1(i)+theta2(i))];
        %y cordinate of mass 1 and 2 at current time step
        Ycoord=[0,l1*cos(theta1(i)),l1*cos(theta1(i))+l2*cos(theta1(i)+theta2(i))];
        %update markers to reflect x and y cords
        set(h1,'XData',Xcoord,'YData',Ycoord);
        drawnow;
        %get current figure (gcf) and save to the movie 'F'
    end
    F(i) = getframe(gcf);
end