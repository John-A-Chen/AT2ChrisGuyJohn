classdef RepairedRobotSimulation
    properties
        totalTime                                                        % store the total elapsed time
        r;                                                               % linearUR3e
    end
    %%
    methods
        function self = RepairedRobotSimulation()
            tic; % Start the timer
            self.totalTime = tic;
            clf;                                                         % Constructor (this is like main)
            axis equal;
            hold on;
            view(3);
            axis ([-5 4 -3 3 0 3]);
            xlim([-5 4]); ylim([-3 3]); zlim([0 3]);
            self.r = LinearUR3e(transl(-0.05, 0, 0.5));                  % create the LinearUR3e robot
            self.setupEnvironment();                                     % Jack Black Lyrics ♥
            self.BrickMover();                                           % Bricks and whatnot ♥

            disp('ta-da here is your 3x3 wall, please come again!');
            elapsedTime = toc(self.totalTime);                           % total elapsed time
            disp(['Total elapsed time: ', num2str(elapsedTime), ' seconds']);
        end

        function setupEnvironment(~)
            tic
            % Setup the environment by placing objects
            surf([-5,-5;4,4], [-3,3;-3,3], [0,0;0,0], 'CData', ...       % create the floor surface
                imread('concrete.jpg'), 'FaceColor', 'texturemap');

            surf([4,4;4,4], [-3,3;-3,3], [3, 3; 0,0], 'CData', ...       % create the wall surface 1
                imread('IMG_7414.jpg'), 'FaceColor', 'texturemap');

            surf([-5,4;-5,4], [3,3;3,3], [3, 3; 0,0], 'CData', ...       % create the wall surface 2
                imread('IMG_7413.jpg'), 'FaceColor', 'texturemap');

            % Place objects
            PlaceObject('fenceAssembly.ply',      [0,     0.75,  -0.965]);
            PlaceObject('tableBrown.ply',         [0,     0,      0.01]);
            PlaceObject('fireExtinguisher.ply',   [0,     1,      0.01]);
            PlaceObject('fireExtinguisher.ply',   [0,    -1,      0.01]);
            PlaceObject('fireExtinguisher.ply',   [1.5,   0,      0.01]);
            PlaceObject('fireExtinguisher.ply',   [-1.5,  0,      0.01]);
            PlaceObject('emergencyStopWall.ply',  [0.9,   0.7,    0.4]);
            PlaceObject('emergencyStopWall.ply',  [0.75,  0.7,    0.4]);
            PlaceObject('emergencyStopWall.ply',  [0.6,   0.7,    0.4]);
            PlaceObject('emergencyStopWall.ply',  [0.45,  0.7,    0.4]);
            PlaceObject('emergencyStopWall.ply',  [0.3,   0.7,    0.4]);
            PlaceObject('emergencyStopWall.ply',  [0.15,  0.7,    0.4]);
            PlaceObject('emergencyStopWall.ply',  [-0.0,  0.7,    0.4]);
            PlaceObject('emergencyStopWall.ply',  [-0.15, 0.7,    0.4]);
            PlaceObject('emergencyStopWall.ply',  [-0.3,  0.7,    0.4]);
            PlaceObject('emergencyStopWall.ply',  [-0.45, 0.7,    0.4]);
            PlaceObject('emergencyStopWall.ply',  [-0.6,  0.7,    0.4]);
            PlaceObject('emergencyStopWall.ply',  [-0.75, 0.7,    0.4]);
            PlaceObject('emergencyStopWall.ply',  [-0.9,  0.7,    0.4]);
            PlaceObject('emergencyStopWall.ply',  [-1.05, 0.7,    0.4]);
            PlaceObject('quarterofaseatlmao.ply', [-2,    0.1,    0.01]);
            PlaceObject('quarterofaseatlmao.ply', [-2,   -0.1,    0.01]);
            PlaceObject('quarterofaseatlmao.ply', [-2,    0.15,   0.01]);
            PlaceObject('quarterofaseatlmao.ply', [-2,   -0.15,   0.01]);
            PlaceObject('emergencyStopButton.ply',[0.8,   0.5,    0.4]);
            PlaceObject('emergencyStopButton.ply',[0.8,   0.25,   0.4]);
            PlaceObject('emergencyStopButton.ply',[0.8,   0.0,    0.4]);
            PlaceObject('emergencyStopButton.ply',[0.8,   -0.25,  0.4]);
            PlaceObject('emergencyStopButton.ply',[0.8,   -0.5,   0.4]);

            elapsedTime = toc;                                           % see the timing
            disp(['elapsed setup time: ', num2str(elapsedTime), ' seconds']);
        end
        function BrickMover(self)
            tic;
            lighting none;

            startPositions = [  -0.10, 0.32, 0.50;                       % Define start and end positions
                -0.20, 0.32, 0.50;
                -0.30, 0.30, 0.50;
                -0.40, 0.30, 0.50;
                -0.50, 0.30, 0.50;
                -0.60, 0.30, 0.50;
                -0.70, 0.30, 0.50;
                -0.80, 0.30, 0.50;
                -0.90, 0.30, 0.50];

            endPositions = [-0.60,   -0.30, 0.50;
                -0.5375, -0.30, 0.50;
                -0.4750, -0.30, 0.50;
                -0.60,   -0.30, 0.5375;
                -0.5375, -0.30, 0.5375;
                -0.4750, -0.30, 0.5375;
                -0.60,   -0.30, 0.575;
                -0.5375, -0.30, 0.575;
                -0.4750, -0.30, 0.575];

            hBricks  = cell(1, size(startPositions, 1));                 % place and object - the brick, at all the cell locations
            for i = 1:size(startPositions, 1)
                hBricks{i} = PlaceObject('tinyBrick.ply', startPositions(i, :));
            end

            brickOrigin = PlaceObject('tinyBrick0.ply', [0,0,0]);        % i made this first brick to reference all the other bricks position
            vertices = get(brickOrigin, 'Vertices');                     % brickVertices stores the brick's 3D coordinates for transformations, reuse, or animation
            delete(brickOrigin);                                         % this deletes the object, not the file

            numSteps = 45;                                               % i have 45 steps because people all use 30 and 50 but i don't like 40
            disp("welcome to unpaidworker")

            for i = 1:size(startPositions, 1)
                whereRobot = self.r.model.getpos();                      % sorts out all the angles of the robot's joints
                startTransform = transl(startPositions(i, :)) * trotx(pi);   % Inv Kin from the start locations of a brick (how to get to start positions)
                startJointAngles = self.r.model.ikcon(startTransform);       % ikcon works with joint angles but i'm not too sure why
                qmatrix = jtraj(whereRobot, startJointAngles, numSteps);     % jtraj finds a way to move the robot from (where to where), quintic poly > trapezoid because its not jerky, but its slower to run

                % this for loop is used to animate the movement as described ^ (the robot is empty, and moves to a place where it can pick up brick)
                for step = 1:numSteps                                    
                    self.r.model.animate(qmatrix(step, :));
                    pause(0.05);                                         % pause between each movement
                end

                t2 = eye(4) * transl(endPositions(i, :)) * trotx(pi);    % Inv Kin from the end locations of a brick (where to go from end pos)
                q2 = self.r.model.ikcon(t2);                             % this part is the same as the previous part but with other points
                qmatrix = jtraj(startJointAngles, q2, numSteps);

                for step = 1:numSteps                                    % animation moves brick (the robot picks up a brick, then moves it to where it needs to go)
                    self.r.model.animate(qmatrix(step, :));
                    whereEndEffector = self.r.model.fkine(qmatrix(step, :)); % fkine is used to compute pose - a 4x4 matrix (of the end effector here)
                    trVertices = [vertices, ones(size(vertices, 1), 1)] * whereEndEffector.T'; % since vertices are stored, they can be transformed (to stick onto the End Effector)

                    set(hBricks{i}, 'Vertices', trVertices(:, 1:3));     % change each brick's new vertices data
                    drawnow();
                    pause(0.05);
                end
                
                disp('end effector is at:');
                disp(whereEndEffector.t);

                % bricklayingSpeedrunAny
                progressPercentage = (i / 9) * 100;                      % calculate progress as a percentage to display the progress
                disp(['bricklaying progress: ', num2str(progressPercentage), '%']);
                delete(hBricks{i});                                      % remove the brick stuck on the end effector
                PlaceObject('tinyBrick.ply', endPositions(i, :));        % replace it with a brick in end position
            end

            % go home
            home = transl(0.05, 0, 1.18); 
            qHome = self.r.model.ikcon(home);
            tHome = jtraj(whereRobot, qHome, numSteps);
            for step = 1:numSteps
                self.r.model.animate(tHome(step, :));
                drawnow();
                pause(0.05);
            end

            elapsedTime = toc;                                           % see the timing
            disp(['elapsed simulation time: ', num2str(elapsedTime), ' seconds']);
        end
    end
end