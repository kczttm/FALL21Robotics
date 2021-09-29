function plotarm(robot)
% uses defineRobot and plot arm config

% radius of the link (as cylinders)
radius = .01;
robot_rb=defineRobot(robot,radius);
hold on;show(robot_rb,robot.q,'Collision','on');
view(-360.001,90);
axis('square');
xlabel('x-axis');ylabel('y-axis');
robot.q = robot.q*(180/pi);
title(sprintf('Planar RRR arm joint configuration in degrees [%.2f,%.2f,%.2f]',...
    robot.q(1),robot.q(2),robot.q(3)));
hold off
end