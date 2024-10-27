classdef johna1test
    properties
        totalTime
        r; % Kuka Titan
        r2; % UR3e
    end
    methods
        function self = johna1test()
            tic;
            self.totalTime = tic;
            clf;
            axis equal;
            hold on;
            view(3);
            axis([-3 3 -3 3 0 3]);
            self.r = KukaTitan(transl(-0.05, 0, 0.01));
            self.setupEnvironment();
            self.BotBuilder();
            self.r2 = UR3e(transl(0, -2, 0));
            self.BrickMover(); % Call BrickMover for the UR3e
            disp('ta-da here is your 3x3 wall, please come again!');
            elapsedTime = toc(self.totalTime);
            disp(['Total elapsed time: ', num2str(elapsedTime), ' seconds']);
        end

        function setupEnvironment(~)
            tic
            surf([-3,-3;3,3], [-3,3;-3,3], [0,0;0,0], 'CData', imread('DSC03374.jpg'), 'FaceColor', 'texturemap');
            surf([3,3;3,3], [-3,3;-3,3], [5, 5; 0,0], 'CData', imread('DSC03375.jpg'), 'FaceColor', 'texturemap');
            surf([-3,3;-3,3], [3,3;3,3], [5, 5; 0,0], 'CData', imread('DSC03376.jpg'), 'FaceColor', 'texturemap');

            PlaceObject('fireExtinguisher.ply', [0, -1, 0.01]);
            PlaceObject('fireExtinguisher.ply', [1.5, 0, 0.01]);
            PlaceObject('fireExtinguisher.ply', [-1.5, 0, 0.01]);
            elapsedTime = toc;
            disp(['elapsed setup time: ', num2str(elapsedTime), ' seconds']);
        end

        function BotBuilder(self)
            tic;
            lighting flat;

            startPositions = [
                2.0, 0.0, 0.05 + 1;
                1.2470, 1.5637, 0.05 + 1;
                -0.4450, 1.9021, 0.05 + 1;
                -1.8019, 0.8678, 0.05 + 1;
                -1.8019, -0.8678, 0.05 + 1;
                -0.4450, -1.9021, 0.05 + 1;
                1.2470, -1.5637, 0.05 + 1];

            endPositions = [
                0, -2, 0;
                0, -2, 0.15185;
                -0.24355, -2, 0.15185;
                -0.45675, -2, 0.15185;
                -0.45675, -2.11235, 0.15185;
                -0.45675, -2.11235, 0.0665;
                -0.45675, -2.19425, 0.0665];

            % hLinks = cell(1, 7);
            % 
            % for i = 1:7
            %     hLinks{i} = PlaceObject(['ur3eLink', num2str(i-1), 'a.ply'], startPositions(i, :));
            % end

            hLink0  = cell(1, size(startPositions, 1));                
            for i = 1:size(startPositions, 1)
                hLink0{i} = PlaceObject('UR3eLink0a.ply', startPositions(i, :));
            end
            Link0Origin = PlaceObject('UR3eLink0a.ply', [0,0,0]);       
            vertices0 = get(Link0Origin, 'Vertices');                     
            delete(Link0Origin);     
            
            hLink1  = cell(1, size(startPositions, 1));                
            for i = 1:size(startPositions, 1)
                hLink1{i} = PlaceObject('UR3eLink1a.ply', startPositions(i, :));
            end
z
            numSteps = 50;
            disp("Welcome to Unpaid Worker");

            for i = 1:7
                whereRobot = self.r.model.getpos();

                startTransform = transl(startPositions(i, :)) * trotx(pi);
                startJointAngles = self.r.model.ikcon(startTransform);
                qmatrix = jtraj(whereRobot, startJointAngles, numSteps);

                for step = 1:numSteps
                    self.r.model.animate(qmatrix(step, :));
                    pause(0.05);
                end

                disp(['Picking up link ', num2str(i)]);

                endTransform = transl(endPositions(i, :)) * trotx(pi);
                qEnd = self.r.model.ikcon(endTransform);
                qmatrix = jtraj(startJointAngles, qEnd, numSteps);

                for step = 1:numSteps
                    self.r.model.animate(qmatrix(step, :));
                    pause(0.05);
                end

                disp(['Placing link ', num2str(i)]);

                link0Vertices = get(hLink0{i}, 'Vertices');
                newVertices = link0Vertices + (endPositions(i, :) - startPositions(i, :));
                set(hLink0{i}, 'Vertices', newVertices);

                link1Vertices = get(hLink1{i}, 'Vertices');
                newVertices = link1Vertices + (endPositions(i, :) - startPositions(i, :));
                set(hLink1{i}, 'Vertices', newVertices);

                link2Vertices = get(hLink2{i}, 'Vertices');
                newVertices = link2Vertices + (endPositions(i, :) - startPositions(i, :));
                set(hLink2{i}, 'Vertices', newVertices);

                link3Vertices = get(hLink3{i}, 'Vertices');
                newVertices = link3Vertices + (endPositions(i, :) - startPositions(i, :));
                set(hLink3{i}, 'Vertices', newVertices);

                link4Vertices = get(hLink4{i}, 'Vertices');
                newVertices = link4Vertices + (endPositions(i, :) - startPositions(i, :));
                set(hLink4{i}, 'Vertices', newVertices);

                link5Vertices = get(hLink5{i}, 'Vertices');
                newVertices = link5Vertices + (endPositions(i, :) - startPositions(i, :));
                set(hLink5{i}, 'Vertices', newVertices);

                link6Vertices = get(hLink6{i}, 'Vertices');
                newVertices = link6Vertices + (endPositions(i, :) - startPositions(i, :));
                set(hLink6{i}, 'Vertices', newVertices);
            end

            home = transl(0.6, 0, 4.137);
            qHome = self.r.model.ikcon(home);
            tHome = jtraj(whereRobot, qHome, numSteps);

            for step = 1:numSteps
                self.r.model.animate(tHome(step, :));
                drawnow();
                pause(0.05);
            end

            elapsedTime = toc;
            disp(['Elapsed simulation time: ', num2str(elapsedTime), ' seconds']);
        end

        function BrickMover(self)
            tic;
            lighting none;

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

            hBricks = cell(1, size(startPositions, 1));
            for i = 1:size(startPositions, 1)
                hBricks{i} = PlaceObject('tinyBrick.ply', startPositions(i, :));
            end

            brickOrigin = PlaceObject('tinyBrick0.ply', [0, 0, 0]);
            vertices = get(brickOrigin, 'Vertices');
            delete(brickOrigin);

            numSteps = 45;
            disp("Welcome to Unpaid Worker")

            for i = 1:size(startPositions, 1)
                whereRobot = self.r2.model.getpos();
                startTransform = transl(startPositions(i, :)) * trotx(pi);
                startJointAngles = self.r2.model.ikcon(startTransform);
                qmatrix = jtraj(whereRobot, startJointAngles, numSteps);

                for step = 1:numSteps
                    self.r2.model.animate(qmatrix(step, :));
                    pause(0.05);
                end

                t2 = eye(4) * transl(endPositions(i, :)) * trotx(pi);
                q2 = self.r2.model.ikcon(t2);
                qmatrix = jtraj(startJointAngles, q2, numSteps);

                for step = 1:numSteps
                    self.r2.model.animate(qmatrix(step, :));
                    whereEndEffector = self.r2.model.fkine(qmatrix(step, :));
                    trVertices = [vertices, ones(size(vertices, 1), 1)] * whereEndEffector.T';
                    set(hBricks{i}, 'Vertices', trVertices(:, 1:3));
                    drawnow();
                    pause(0.05);
                end

                disp('End effector is at:');
                disp(whereEndEffector.t);

                progressPercentage = (i / size(startPositions, 1)) * 100;
                disp(['Bricklaying progress: ', num2str(progressPercentage), '%']);
                delete(hBricks{i});
                PlaceObject('tinyBrick.ply', endPositions(i, :));
            end

            % Return home
            home = transl(0.05, 0, 1.18); 
            qHome = self.r2.model.ikcon(home);
            tHome = jtraj(whereRobot, qHome, numSteps);
            for step = 1:numSteps
                self.r2.model.animate(tHome(step, :));
                drawnow();
                pause(0.05);
            end

            elapsedTime = toc;
            disp(['Elapsed simulation time: ', num2str(elapsedTime), ' seconds']);
        end
    end
end
