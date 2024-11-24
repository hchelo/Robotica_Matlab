function z = lasercordlocal(car,robot)
delta = car(1:2)'-[robot.x;robot.y];
ang = car(3)*pi/180-pi/2;
llocalxy = [cos(robot.tita) sin(robot.tita);-sin(robot.tita) cos(robot.tita)]*(delta);
z = [llocalxy;ang];

