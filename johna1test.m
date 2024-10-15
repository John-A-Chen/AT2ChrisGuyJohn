classdef johna1test
    properties
        totalTime
        r;
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

            disp('ta-da here is your 3x3 wall, please come again!');
            elapsedTime = toc(self.totalTime);
            disp(['Total elapsed time: ', num2str(elapsedTime), ' seconds']);
        end

        function setupEnvironment(~)
            tic
            surf([-3,-3;3,3], [-3,3;-3,3], [0,0;0,0], 'CData', imread('concrete.jpg'), 'FaceColor', 'texturemap');
            surf([3,3;3,3], [-3,3;-3,3], [3, 3; 0,0], 'CData', imread('IMG_7414.jpg'), 'FaceColor', 'texturemap');
            surf([-3,3;-3,3], [3,3;3,3], [3, 3; 0,0], 'CData', imread('IMG_7413.jpg'), 'FaceColor', 'texturemap');

            PlaceObject('fireExtinguisher.ply', [0, 1, 0.01]);
            PlaceObject('fireExtinguisher.ply', [0, -1, 0.01]);
            PlaceObject('fireExtinguisher.ply', [1.5, 0, 0.01]);
            PlaceObject('fireExtinguisher.ply', [-1.5, 0, 0.01]);
            elapsedTime = toc;
            disp(['elapsed setup time: ', num2str(elapsedTime), ' seconds']);
        end

        function BotBuilder(self)
            tic;
            lighting gouraud;

            startPositions = [
                1.5, 1.8, 0.05;
                -1.2, 2.1, 0.05;
                0.9, -1.9, 0.05;
                -2.5, 0.7, 0.05;
                2.3, -0.9, 0.05;
                -1.8, -1.6, 0.05
                ];

            endPositions = [
                0, 1, 0.1807;
                -0.6127, 1, 0.1807;
                -1.1843, 1, 0.1807;
                -1.1843, 1, 0.35485;
                -1.1843, 1, 0.4747;
                -1.1843, 1, 0.59125
                ];

            hLinks = cell(1, 6);

            for i = 1:6
                hLinks{i} = PlaceObject(['UR10eLink', num2str(i-1), '.ply'], startPositions(i, :));
            end

            numSteps = 50;
            disp("Welcome to Unpaid Worker");

            for i = 1:6
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

                linkVertices = get(hLinks{i}, 'Vertices'); 
                newVertices = linkVertices + (endPositions(i, :) - startPositions(i, :));
                set(hLinks{i}, 'Vertices', newVertices);
            end

            home = transl(1.1, 0, 4.137);
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

    end
end