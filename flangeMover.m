function flangeMover(self)
startPositions = [-1, 1, 1];

endPositions = [2,-2, 1];

hflange  = cell(1, size(startPositions, 1));
for i = 1:size(startPositions, 1)
    hflange{i} = PlaceObject('flange.ply', startPositions(i, :));
end

flangeOrigin = PlaceObject('flange0.ply', [0,0,0]);
vertices = get(flangeOrigin, 'Vertices');
delete(flangeOrigin);

numSteps = 50;
disp("gonna weld this flange")

for i = 1:size(startPositions, 1)
    whereRobot = self.r.model.getpos();
    startTransform = transl(startPositions(i, :)) * trotx(pi);
    startJointAngles = self.r.model.ikcon(startTransform);
    qmatrix = jtraj(whereRobot, startJointAngles, numSteps);

    for step = 1:numSteps
        self.r.model.animate(qmatrix(step, :));
        pause(0.05);
    end

    t2 = eye(4) * transl(endPositions(i, :)) * trotx(pi);
    q2 = self.r.model.ikcon(t2);
    qmatrix = jtraj(startJointAngles, q2, numSteps);

    for step = 1:numSteps
        self.r.model.animate(qmatrix(step, :));
        whereEndEffector = self.r.model.fkine(qmatrix(step, :));
        trVertices = [vertices, ones(size(vertices, 1), 1)] * whereEndEffector.T';

        set(hflange{i}, 'Vertices', trVertices(:, 1:3));
        drawnow();
        pause(0.05);
    end
    delete(hflange{i});
    PlaceObject('flange.ply', endPositions(i, :));
end
end
