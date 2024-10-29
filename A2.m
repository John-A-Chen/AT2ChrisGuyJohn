classdef A2 < handle
    properties
        totalTime
        S = 5;
        ur3e;
        q_ur3e;
        titan;
        q_titan;
        x1;
        x2;
        stopped = false;
        a;
        s = gobjects(1,12);
        b = gobjects(1,4);
<<<<<<< HEAD
        centerPoints = [
            0, 0, 0;       % Link 1
            0, 0, 0.14;    % Link 2
            0, 0, 0.14;    % Link 3
            0, 0, 0;       % Link 4
            0, 0, 0;       % Link 5
            0, 0, 0;       % Link 6
            ];

        radii = [
            0.2, 0.2, 0.2;   % Link 1
            0.28, 0.2, 0.28; % Link 2
            0.4, 0.2, 0.28;  % Link 3
            0.2, 0.2, 0.2;   % Link 4
            0.2, 0.2, 0.2;   % Link 5
            0.2, 0.2, 0.2;   % Link 6
            ];

=======
        f = gobjects(1,20);
>>>>>>> 5cff1ad556ad43649073a8ebcb69d845a0e6a5aa
    end

    methods
        function self = A2()
            tic;
            self.totalTime = tic;
            clf;
            axis equal;
            hold on;
            view(3);
            axis ([-self.S, self.S, -self.S, self.S, 0, self.S]);
            xlim([-self.S, self.S]);
            ylim([-self.S, self.S]);
            zlim([0, self.S]);
            % self.a = arduino;
            self.ur3e = UR3e(transl(2.5, 1.75, 0.925));
            self.q_ur3e = zeros(1,6);
            self.titan = KukaTitan(transl(0, 0, 0.05));
            self.q_titan = zeros(1,6);
            self.x1 = zeros(3,1);
            self.x2 = zeros(3,1);
            self.setupEnvironment();
            elapsedTime = toc(self.totalTime);
            disp(['Total elapsed time: ', num2str(elapsedTime), ' seconds']);
            self.startUI();
            self.defineObstacle(2,2,2,-0.5,0.5,-0.5,0.5);
        end

        function setupEnvironment(self)
            surf([-self.S, -self.S; self.S, self.S], ...
                [-self.S, self.S; -self.S, self.S], [0, 0; 0, 0], ...
                'CData', imread('concrete.jpg'), 'FaceColor', 'texturemap');
            surf([self.S, self.S; self.S, self.S], ...
                [-self.S, self.S; -self.S, self.S], [self.S, self.S; 0, 0], ...
                'CData', imread('IMG_7414.jpg'), 'FaceColor', 'texturemap');
            surf([-self.S, self.S; -self.S, self.S], ...
                [self.S, self.S; self.S, self.S], [self.S, self.S; 0, 0], ...
                'CData', imread('IMG_7413.jpg'), 'FaceColor', 'texturemap');
            PlaceObject('environment.PLY',[0,0,0]);
            light("Style","local","Position",[-10 -10 3]);
        end

        function startUI(self, ~)
            delete(self.b(1));
            delete(self.b(2));
            self.b(1) = uicontrol('Style','pushbutton','String','Free Control', ...
                'Position', [100 200 100 40],'Callback', @self.freeControl);
            self.b(2) = uicontrol('Style','pushbutton','String','Sequence', ...
                'Position', [100 150 100 40],'Callback', @self.sequence);
            self.b(3) = uicontrol('Style','pushbutton','String','Path Planning', ...
                'Position', [100 100 100 40],'Callback', @self.pathPlanning);
        end

        function freeControl(self, source, ~)
            self.stopped = false;
            delete(source);
            delete([self.b(1); self.b(2); self.b(3); self.b(4); self.s(1); self.s(2); ...
                self.s(3); self.s(4); self.s(5); self.s(6); self.s(7); self.s(8); ...
                self.s(9); self.s(10); self.s(11); self.s(12); ]);
            sliderProperties = struct('Style', 'slider', 'Value', 0, ...
                'SliderStep', [0.01 0.1], 'Callback', @self.updateJoints);
            self.s(1) = uicontrol(sliderProperties, ...
                'Position', [10 120 300 20], 'String', 'tq1', ...
                'Min', -150 ,  'Max', 150, "Value", self.q_titan(1) * 180/pi);
            self.f(1) = uilabel("Titan q1", 'Position', [310 120 300 20]);
            self.s(2) = uicontrol(sliderProperties, ...
                'Position', [10 100 300 20], 'String', 'tq2', ...
                'Min', -40 ,  'Max', 107.5, "Value", self.q_titan(2) * 180/pi);
            self.s(3) = uicontrol(sliderProperties, ...
                'Position', [10 80 300 20], 'String', 'tq3', ...
                'Min', -200 ,  'Max', 55, "Value", self.q_titan(3) * 180/pi);
            self.s(4) = uicontrol(sliderProperties, ...
                'Position', [10 60 300 20], 'String', 'tq4', ...
                'Min', -350,'Max', 350, "Value", self.q_titan(4) * 180/pi);
            self.s(5) = uicontrol(sliderProperties, ...
                'Position', [10 40 300 20], 'String', 'tq5', ...
                'Min', -118,'Max', 118, "Value", self.q_titan(5) * 180/pi);
            self.s(6) = uicontrol(sliderProperties, ...
                'Position', [10 20 300 20], 'String', 'tq6', ...
                'Min', -350,'Max', 350, "Value", self.q_titan(6) * 180/pi);

            self.s(7) = uicontrol(sliderProperties, ...
                'Position', [1200 120 300 20], 'String', 'uq1', ...
                'Min', -360 ,  'Max', 360, "Value", self.q_ur3e(1) * 180/pi);
            self.s(8) = uicontrol(sliderProperties, ...
                'Position', [1200 100 300 20], 'String', 'uq2', ...
                'Min', -90 ,  'Max', 90, "Value", self.q_ur3e(2) * 180/pi);
            self.s(9) = uicontrol(sliderProperties, ...
                'Position', [1200 80 300 20], 'String', 'uq3', ...
                'Min', -170 ,  'Max', 170, "Value", self.q_ur3e(3) * 180/pi);
            self.s(10) = uicontrol(sliderProperties, ...
                'Position', [1200 60 300 20], 'String', 'uq4', ...
                'Min', -360,'Max', 360, "Value", self.q_ur3e(4) * 180/pi);
            self.s(11) = uicontrol(sliderProperties, ...
                'Position', [1200 40 300 20], 'String', 'uq5', ...
                'Min', -360,'Max', 360, "Value", self.q_ur3e(5) * 180/pi);
            self.s(12) = uicontrol(sliderProperties, ...
                'Position', [1200 20 300 20], 'String', 'uq6', ...
                'Min', -360,'Max', 360, "Value", self.q_ur3e(6) * 180/pi);

            self.b(3) = uicontrol('Style','pushbutton','String','E-Stop', ...
                'Position', [110 150 100 50],'Callback', @self.eStop);
            self.b(4) = uicontrol('Style','pushbutton','String','Sequence', ...
                'Position', [110 200 100 50],'Callback', @self.sequence);
        end

        function sequence(self, source, ~)
            self.stopped = false;
            delete(source);
            delete([self.b(1); self.b(2); self.b(3); self.b(4); self.s(1); self.s(2); ...
                self.s(3); self.s(4); self.s(5); self.s(6); self.s(7); self.s(8); ...
                self.s(9); self.s(10); self.s(11); self.s(12)]);

            self.b(3) = uicontrol('Style','pushbutton','String','E-Stop', ...
                'Position', [110 150 100 50],'Callback', @self.eStop);

            self.b(2) = uicontrol('Style','pushbutton','String','Free Control', ...
                'Position', [110 200 100 50],'Callback', @self.freeControl);

            

            self.DLS();
        end

        function pathPlanning(self, source, ~)
            self.stopped = false;
            delete(source);
            delete([self.b(1); self.b(2); self.b(3); self.b(4); self.s(1); self.s(2); ...
                self.s(3); self.s(4); self.s(5); self.s(6); self.s(7); self.s(8); ...
                self.s(9); self.s(10); self.s(11); self.s(12)]);
            self.b(3) = uicontrol('Style','pushbutton','String','E-Stop', ...
                'Position', [110 150 100 50],'Callback', @self.eStop);
            self.b(4) = uicontrol('Style','pushbutton','String','Confirm', ...
                'Position', [110 200 100 50],'Callback', @self.updatePath);

            sliderProperties = struct('Style', 'slider', 'Value', 0, ...
                'SliderStep', [0.01 0.1], 'Callback', @self.updatePath);

            self.s(1) = uicontrol(sliderProperties, ...
                'Position', [10 120 300 20], 'String', 'x1', ...
                'Min', -150 ,  'Max', 150, "Value", self.x1(1) * 180/pi);
            self.s(2) = uicontrol(sliderProperties, ...
                'Position', [10 100 300 20], 'String', 'y1', ...
                'Min', -40 ,  'Max', 107.5, "Value", self.x1(2) * 180/pi);
            self.s(3) = uicontrol(sliderProperties, ...
                'Position', [10 80 300 20], 'String', 'z1', ...
                'Min', -200 ,  'Max', 55, "Value", self.x1(3) * 180/pi);

            self.s(7) = uicontrol(sliderProperties, ...
                'Position', [1200 120 300 20], 'String', 'x2', ...
                'Min', -150 ,  'Max', 150, "Value", self.x2(1) * 180/pi);
            self.s(8) = uicontrol(sliderProperties, ...
                'Position', [1200 100 300 20], 'String', 'y2', ...
                'Min', -40 ,  'Max', 107.5, "Value", self.x2(2) * 180/pi);
            self.s(9) = uicontrol(sliderProperties, ...
                'Position', [1200 80 300 20], 'String', 'z2', ...
                'Min', -200 ,  'Max', 55, "Value", self.x2(3) * 180/pi);
        end

        function DLS(self)
            t = 10;                                                         % Total time in seconds
            steps = 200;                                                    % No. of steps
            deltaT = t/steps;                                               % Discrete time step
            deltaTheta = 4*pi/steps;                                        % Small angle change
            qMatrix = zeros(steps,6);                                       % Assign memory for joint angles
            x = zeros(3,steps);                                             % Assign memory for trajectory
            m = zeros(1,steps);                                             % For recording measure of manipulability
            errorValue = zeros(3,steps);                                    % For recording velocity error
            lambda = 0;
            lambdaMax = 0.05;
            epsilon = 0.5;

            for i = 1:steps
                x(:,i) = [1.5*cos(deltaTheta*i) + 0.45*cos(deltaTheta*i)
                    1.5*sin(deltaTheta*i) + 0.45*cos(deltaTheta*i)
                    3];
            end

            qMatrix(1,:) = self.titan.model.ikcon(transl(x(:,1)));
            self.titan.model.animate(qMatrix(1,:));

            for i = 1:steps-1
                T = self.titan.model.fkine(qMatrix(i,:)).T;                 % End-effector transform at current joint state
                xdot = (x(:,i+1)-T(1:3,4));                                 % Velocity to reach next waypoint
                J = self.titan.model.jacob0(qMatrix(i,:));                  % Get Jacobian at current state (use jacob0)
                J = J(1:3,:);                                               % Take only first 3 rows
                m(:,i) = sqrt(det(J*J'));                                   % Measure of Manipulability
                if m(:,i) > epsilon
                    lambda = 0;
                else
                    lambda = (1 - (m(:,i)/epsilon)^2) * lambdaMax;
                end
                qdot = J'*inv(J*J' + lambda * eye(3))*xdot;                 % Solve the RMRC equation
                errorValue(:,i) = xdot - J*qdot;                            % Velocity error
                qMatrix(i+1,:)= qMatrix(i,:) + (qdot)';                     % Update the joint state
                self.titan.model.plot(qMatrix(i+1,:));
                disp(i)
                self.q_titan = qMatrix(i+1,:);
                if self.stopped
                    break
                end
                % if readDigitalPin(self.a, "D23")
                %     eStop()
                %     break
                % end
            end
        end

        function updateJoints(self, source, ~)                              % issue here is that source changes the number
            sliderValue1 = get(source, 'Value');                            % for both of them so they both dance
            sliderValue2 = get(source, 'Value');
            disp(source);
            name = source.String;
            number = str2num(['uint8(',name(3),')']);
            if strcmp(name(1),'t')
                q1 = self.q_titan;
                q1(number) = sliderValue1 * pi/180;
                self.q_titan = q1;
                self.titan.model.animate(q1);
            elseif strcmp(name(1),'u')
                q2 = self.q_ur3e;
                q2(number) = sliderValue2 * pi/180;
                self.q_ur3e = q2;
                self.ur3e.model.animate(q2);
            end
            % q1 = self.q_titan;
            % q2 = self.q_ur3e;
            % q1(number) = sliderValue1 * pi/180;
            % q2(number) = sliderValue2 * pi/180;
            % self.q_titan = q1;
            % self.q_ur3e = q2;
            % self.titan.model.animate(q1);
            % self.ur3e.model.animate(q2);
        end

        function updatePath(self, source, ~)
            sliderValue1 = get(source, 'Value');
            disp(source)
            disp(self.q_titan);
        end

        function eStop(self, source, ~)
            % disp(so)
            disp("E has been stopped");
            self.stopped = true;
            delete(source);
            delete([self.b(1); self.b(2); self.b(3); self.b(4); self.s(1); self.s(2); ...
                self.s(3); self.s(4); self.s(5); self.s(6); self.s(7); self.s(8); ...
                self.s(9); self.s(10); self.s(11); self.s(12)]);

            self.b(1) = uicontrol('Style','pushbutton','String','Free Control', ...
                'Position', [100 150 100 50],'Callback', @self.freeControl);
            self.b(2) = uicontrol('Style','pushbutton','String','Sequence', ...
                'Position', [100 200 100 50],'Callback', @self.sequence);
            self.b(3) = uicontrol('Style','pushbutton','String','Path Planning', ...
                'Position', [100 100 100 50],'Callback', @self.pathPlanning);
        end
        function defineObstacle(self, obsX, obsY, obsZ, a, b, c, d)
            %% make the cube
            % [Y,Z] = meshgrid(-0.5:0.05:0.5,-0.5:0.05:0.5);
            [Y,Z] = meshgrid(a:0.05:b,c:0.05:d);
            sizeMat = size(Y);
            X = repmat(0.5,sizeMat(1),sizeMat(2));
            oneSideOfCube_h = surf(X,Y,Z);
            cubePoints = [X(:),Y(:),Z(:)];
            cubePoints = [ cubePoints ...
                ; cubePoints * rotz(pi/2)...
                ; cubePoints * rotz(pi) ...
                ; cubePoints * rotz(3*pi/2) ...
                ; cubePoints * roty(pi/2) ...
                ; cubePoints * roty(-pi/2)];
            % cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
            cubePoints = cubePoints + repmat([obsX,obsY,obsZ],size(cubePoints,1),1);
            cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
            % set(cube_h, 'Visible', 'off');
            axis equal;
        end
        
    end
end
