link(1) = Link('d',0.15185,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
link(2) = Link('d',0,'a',-0.24355,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
link(3) = Link('d',0,'a',-0.2132,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
link(4) = Link('d',0.13105,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
link(5) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
link(6) = Link('d',	0.0921,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

self.model = SerialLink(link,'name','ur3e');

startPositions = [  
    -0.10, 0.32, 0.50;
    -0.20, 0.32, 0.50;
    -0.30, 0.30, 0.50;
    -0.40, 0.30, 0.50;
    -0.50, 0.30, 0.50;
    -0.60, 0.30, 0.50;
    -0.70, 0.30, 0.50;
    -0.80, 0.30, 0.50;
    -0.90, 0.30, 0.50];

endPositions = [
    -0.60, -0.30, 0.50;
    -0.5375, -0.30, 0.50;
    -0.4750, -0.30, 0.50;
    -0.60, -0.30, 0.5375;
    -0.5375, -0.30, 0.5375;
    -0.4750, -0.30, 0.5375;
    -0.60, -0.30, 0.575;
    -0.5375, -0.30, 0.575;
    -0.4750, -0.30, 0.575];

dt = 0.05;

steps = 100;

for i = 1:size(startPositions, 1)
    q = robot.getpos();
    startPose = transl(startPositions(i, :)) * trotx(pi);
    endPose = transl(endPositions(i, :)) * trotx(pi);
    
    for t = 0:dt:2
        T = robot.fkine(q);
        v_e = (tr2delta(T, startPose) / dt)';
        J = robot.jacob0(q);
        q_dot = pinv(J) * v_e;
        q = q + q_dot' * dt;
        robot.plot(q);
        startPose = transl(startPositions(i, :) + (endPositions(i, :) - startPositions(i, :)) * t / 2);
        pause(dt);
    end
end

disp('RMRC sequence completed.');
