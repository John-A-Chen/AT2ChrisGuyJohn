clf;
axis equal;
hold on;
view(3);
axis([-5 4 -3 3 0 3]);
xlim([-5 4]); ylim([-3 3]); zlim([0 3]);

r = KukaTitan(transl(-0.75, 0.5, 0.05));

PlaceObject('environment.PLY', [0, 0, 0]);

startPositions = ...
[-1.0, 3.5, 1.3;
 -1.4, 3.5, 1.3;
 -1.8, 3.5, 1.3];

weldPosition = ...
[2, 2, 1.75;
 2, 2, 1.75;
 2, 2, 1.75];

endPositions = ...
[-0.375, 2, 0.9;
 0.0, 2, 0.9;
 0.375, 2, 0.9];

hflange = cell(1, size(startPositions, 1));
for i = 1:size(startPositions, 1)
    hflange{i} = PlaceObject('flange.ply', startPositions(i, :));
end

flangeOrigin = PlaceObject('flange0.ply', [0, 0, 0]);
vertices = get(flangeOrigin, 'Vertices');
delete(flangeOrigin);

numSteps = 50;
t = linspace(0, 1, numSteps);  % Time vector for LSPB

for i = 1:size(startPositions, 1)
    whereRobot = r.model.getpos();
    startTransform = transl(startPositions(i, :)) * trotx(pi);
    startJointAngles = r.model.ikcon(startTransform);

    % Move to start position using LSPB
    for j = 1:r.model.n
        qStart(:, j) = lspb(whereRobot(j), startJointAngles(j), t);
    end

    for step = 1:numSteps
        r.model.animate(qStart(step, :));
        pause(0.05);
    end

    % Move to weld position using LSPB
    tWeld = eye(4) * transl(weldPosition(i, :)) * trotx(pi);
    qWeld = r.model.ikcon(tWeld);

    for j = 1:r.model.n
        qWeldMatrix(:, j) = lspb(startJointAngles(j), qWeld(j), t);
    end

    for step = 1:numSteps
        r.model.animate(qWeldMatrix(step, :));
        whereEndEffector = r.model.fkine(qWeldMatrix(step, :));

        trVertices = [vertices, ones(size(vertices, 1), 1)] * whereEndEffector.T';
        set(hflange{i}, 'Vertices', trVertices(:, 1:3));
        drawnow();
        pause(0.05);
    end

    % Stay at weld position for 5 seconds
    pause(5);

    % Move to end position using LSPB
    tEnd = eye(4) * transl(endPositions(i, :)) * trotx(pi);
    qEnd = r.model.ikcon(tEnd);

    for j = 1:r.model.n
        qEndMatrix(:, j) = lspb(qWeld(j), qEnd(j), t);
    end

    for step = 1:numSteps
        r.model.animate(qEndMatrix(step, :));
        whereEndEffector = r.model.fkine(qEndMatrix(step, :));

        trVertices = [vertices, ones(size(vertices, 1), 1)] * whereEndEffector.T';
        set(hflange{i}, 'Vertices', trVertices(:, 1:3));
        drawnow();
        pause(0.05);
    end

    delete(hflange{i});
    PlaceObject('flange.ply', endPositions(i, :));
end
