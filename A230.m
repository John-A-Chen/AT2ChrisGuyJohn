classdef A230 < handle
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
        s = gobjects(1,13);
        b = gobjects(1,4);
        path = gobjects(1,100);

        steps;
        total_steps;
        epsilon;
        deltaT;
        iCurrent;

        c;
        r;
        W = diag([1 1 1 0.1 0.1 0.1]);
        x_2;
        x_1;

        trVertices;

    end

    methods
        function self = A230()
            tic;
            self.totalTime = tic;
            clf;
            axis equal;
            hold on;
            view(3);
            axis([-self.S, self.S, -self.S, self.S, 0, self.S]);
            xlim([-self.S, self.S]);
            ylim([-self.S, self.S]);
            zlim([0, self.S]);
            % self.a = arduino;
            self.ur3e = UR3e(transl(2.5, 1.75, 0.925));
            self.q_ur3e = zeros(1,7);
            self.titan = KukaTitan(transl(-0.75,0.5,0.05));
            self.q_titan = zeros(1,6);
            self.x1 = zeros(3,1);
            self.x2 = zeros(3,1);
            self.steps = 1/0.2;
            self.total_steps = 2 * self.steps;
            self.epsilon = 0.1;
            self.deltaT = 0.02;
            self.iCurrent = 1;

            self.c = [2.5; 1.5; 1];
            self.r = [0.6; 0.6; 0.6];
            self.setupEnvironment();
            elapsedTime = toc(self.totalTime);
            disp(['Total elapsed time: ', num2str(elapsedTime), ' seconds']);
            self.startUI();
            % self.elipsoidOnRobotUR3e(3,1,1,0.2);
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
            % ellipsoid(self.c(1),self.c(2),self.c(3),self.r(1),self.r(2),self.r(3));
        end

        function startUI(self, ~)
            delete(self.b(1));
            delete(self.b(2));
            self.b(1) = uicontrol('Style','pushbutton','String','Free Control', ...
                'Position', [100 200 100 40],'Callback', @self.freeControl);
            self.b(2) = uicontrol('Style','pushbutton','String','Sequence', ...
                'Position', [100 150 100 40],'Callback', @self.sequence);
            self.b(3) = uicontrol('Style','pushbutton','String','Path Placement', ...
                'Position', [100 100 100 40],'Callback', @self.pathPlacement);
            self.b(4) = uicontrol('Style','pushbutton','String','Controller', ...
                'Position', [100 50 100 40],'Callback', @self.controller);
        end

        function freeControl(self, source, ~)
            self.stopped = false;
            delete(source);
            delete([self.b(1); self.b(2); self.b(3); self.b(4); self.s(1); self.s(2); ...
                self.s(3); self.s(4); self.s(5); self.s(6); self.s(7); self.s(8); ...
                self.s(9); self.s(10); self.s(11); self.s(12)]);
            sliderProperties = struct('Style', 'slider', 'Value', 0, ...
                'SliderStep', [0.01 0.1], 'Callback', @self.updateJoints);
            self.s(1) = uicontrol(sliderProperties, ...
                'Position', [10 120 300 20], 'String', 'tq1', ...
                'Min', -150 ,  'Max', 150, "Value", self.q_titan(1) * 180/pi);
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
            self.s(13) = uicontrol(sliderProperties, ...
                'Position', [1700 10 300 20], 'String', 'uq7', ...
                'Min', -360,'Max', 360, "Value", self.q_ur3e(7) * 180/pi);

            self.b(3) = uicontrol('Style','pushbutton','String','Back', ...
                'Position', [110 150 100 50],'Callback', @self.eStop);
            self.b(4) = uicontrol('Style','pushbutton','String','Sequence', ...
                'Position', [110 200 100 50],'Callback', @self.sequence);
        end

        function sequence(self, source, ~)
            self.stopped = false;
            delete(source);
            delete([self.b(1); self.b(2); self.b(3); self.b(4); self.s(1); self.s(2); ...
                self.s(3); self.s(4); self.s(5); self.s(6); self.s(7); self.s(8); ...
                self.s(9); self.s(10); self.s(11); self.s(12); self.s(13)]);

            self.b(3) = uicontrol('Style','pushbutton','String','E-Stop', ...
                'Position', [110 150 100 50],'Callback', @self.eStop);

            self.b(2) = uicontrol('Style','pushbutton','String','Free Control', ...
                'Position', [110 200 100 50],'Callback', @self.freeControl);

            % self.DLS();
            % self.DLS2();
            self.rmrc5();
            % self.welding();
            % self.jTRAJICwelding();
        end

        function pathPlacement(self, source, ~)
            self.stopped = false;
            delete(source);
            delete([self.b(1); self.b(2); self.b(3); self.b(4); self.s(1); self.s(2); ...
                self.s(3); self.s(4); self.s(5); self.s(6); self.s(7); self.s(8); ...
                self.s(9); self.s(10); self.s(11); self.s(12); self.s(13)]);

            sliderProperties = struct('Style', 'slider', 'Value', 0, ...
                'SliderStep', [0.01 0.1], 'Callback', @self.updatePath);
            self.s(1) = uicontrol(sliderProperties, ...
                'Position', [10 120 300 20], 'String', 'x1', ...
                'Min', -3 ,  'Max', 3, "Value", self.x1(1));
            self.s(2) = uicontrol(sliderProperties, ...
                'Position', [10 100 300 20], 'String', 'y1', ...
                'Min', -3 ,  'Max', 3, "Value", self.x1(2));
            self.s(3) = uicontrol(sliderProperties, ...
                'Position', [10 80 300 20], 'String', 'z1', ...
                'Min', 0 ,  'Max', 3, "Value", self.x1(3));

            self.s(7) = uicontrol(sliderProperties, ...
                'Position', [1200 120 300 20], 'String', 'x2', ...
                'Min', -3 ,  'Max', 3, "Value", self.x2(1));
            self.s(8) = uicontrol(sliderProperties, ...
                'Position', [1200 100 300 20], 'String', 'y2', ...
                'Min', -3 ,  'Max', 3, "Value", self.x2(2));
            self.s(9) = uicontrol(sliderProperties, ...
                'Position', [1200 80 300 20], 'String', 'z2', ...
                'Min', 0 ,  'Max', 3, "Value", self.x2(3));

            self.b(3) = uicontrol('Style','pushbutton','String','Back', ...
                'Position', [110 150 100 50],'Callback', @self.eStop);

            self.b(2) = uicontrol('Style','pushbutton','String','Play', ...
                'Position', [110 200 100 50],'Callback', @self.play);

            for i = 1:100
                t = i * 0.01;
                x = self.x1(1) * (1 - t) + self.x2(1) * t;
                y = self.x1(2) * (1 - t) + self.x2(2) * t;
                z = self.x1(3) * (1 - t) + self.x2(3) * t;
                v = [x; y; z];
                d = (v-self.c)'*diag(self.r.^-2)*(v-self.c);
                if d < 1
                    v = self.c + (v-self.c) * d^-0.5;
                end
                self.path(i) = plot3(v(1),v(2),v(3),'r*');
            end
        end

        function controller(self, source, ~)
            self.stopped = false;
            delete(source);
            delete([self.b(1); self.b(2); self.b(3); self.b(4); self.s(1); self.s(2); ...
                self.s(3); self.s(4); self.s(5); self.s(6); self.s(7); self.s(8); ...
                self.s(9); self.s(10); self.s(11); self.s(12); self.s(13)]);

            self.b(3) = uicontrol('Style','pushbutton','String','eStop', ...
                'Position', [110 150 100 50],'Callback', @self.eStop);

            id = 1; % Note: may need to be changed if multiple joysticks present
            joy = vrjoystick(id);
            caps(joy) % display joystick information

            uq = zeros(1,6);
            tq = zeros(1,6);
            dt = 0.1;

            % tic;
            while true
                [axes, buttons, povs] = read(joy);
                Kuv = 0.3;
                Ktv = 1;

                ux = Kuv*axes(1);
                uy = -Kuv*axes(2);
                uz = Kuv*(buttons(8)-buttons(7));

                tx = Ktv*axes(3);
                ty = -Ktv*axes(4);
                tz = Ktv*(buttons(5)-buttons(2));

                dux = [ux;uy;uz;0;0;0];
                dtx = [tx;ty;tz;0;0;0];

                lambda = 0.1;
                uJ = self.ur3e.model.jacob0(uq);
                uJinv_dls = inv((uJ'*uJ)+lambda^2*eye(6))*uJ';
                duq = uJinv_dls*dux;
                uq = uq + duq'*dt;
                self.ur3e.model.animate(uq);
                self.q_ur3e = uq;

                disp(tq);
                tJ = self.titan.model.jacob0(tq);
                tJinv_dls = inv((tJ'*tJ)+lambda^2*eye(6))*tJ';
                dtq = tJinv_dls*dtx;
                tq = tq + dtq'*dt;
                self.titan.model.animate(tq);
                self.q_titan = tq;

                drawnow

                if self.stopped
                    break
                end
            end
        end

        % function play(self, source, ~)
        %     self.stopped = false;
        %     delete(source);
        %     delete([self.b(1); self.b(2); self.b(3); self.b(4); self.s(1); self.s(2); ...
        %         self.s(3); self.s(4); self.s(5); self.s(6); self.s(7); self.s(8); ...
        %         self.s(9); self.s(10); self.s(11); self.s(12); self.s(13)]);
        %
        %     self.b(3) = uicontrol('Style','pushbutton','String','eStop', ...
        %         'Position', [110 150 100 50],'Callback', @self.eStop);
        %
        %     t = 10;                                                         % Total time in seconds
        %     steps = 100;                                                    % No. of steps
        %     deltaT = t/steps;                                               % Discrete time step
        %     qMatrix = zeros(steps,6);                                      % Assign memory for joint angles
        %     x = zeros(3,steps);                                             % Assign memory for trajectory
        %     m = zeros(1,steps);                                             % For recording measure of manipulability
        %     errorValue = zeros(3,steps);                                    % For recording velocity error
        %     lambda = 0;
        %     lambdaMax = 0.05;
        %     epsilon = 0.5;
        %
        %     for i = 1:steps
        %         x(1,i) = self.path(i).XData;
        %         x(2,i) = self.path(i).YData;
        %         x(3,i) = self.path(i).ZData;
        %         % q = self.titan.model.ikcon(transl(x,y,z));
        %         % self.titan.model.animate(q);
        %         % drawnow;
        %     end
        %     i = self.iCurrent;
        %     qMatrix(i,:) = self.titan.model.ikcon(transl(x(:,i)'), [0,0,0.5,0,0,0]);
        %     self.titan.model.animate(qMatrix(i,:));
        %     drawnow
        %
        %     % for i = 1:steps-1
        %     while i <= 99
        %         T = self.titan.model.fkine(qMatrix(i,:)).T;                 % End-effector transform at current joint state
        %         xdot = (x(:,i+1)-T(1:3,4));                                 % Velocity to reach next waypoint
        %         J = self.titan.model.jacob0(qMatrix(i,:));                  % Get Jacobian at current state (use jacob0)
        %         J = J(1:3,:);                                               % Take only first 3 rows
        %         m(:,i) = sqrt(det(J*J'));                                   % Measure of Manipulability
        %         if m(:,i) > epsilon
        %             lambda = 0;
        %         else
        %             lambda = (1 - (m(:,i)/epsilon)^2) * lambdaMax;
        %         end
        %         qdot = J'*inv(J*J' + lambda * eye(3))*xdot;                 % Solve the RMRC equation
        %         errorValue(:,i) = xdot - J*qdot;                            % Velocity error
        %         qMatrix(i+1,:)= qMatrix(i,:) + (qdot)';                     % Update the joint state
        %         self.titan.model.animate(qMatrix(i+1,:));
        %         drawnow
        %         self.q_titan = qMatrix(i+1,:);
        %         if self.stopped
        %             self.iCurrent = i;
        %             break
        %
        %         end
        %         i = i + 1;
        %         % if readDigitalPin(self.a, "D23")
        %         %     break
        %         % end
        %     end
        %
        %
        %
        %     if ~self.stopped
        %         self.iCurrent = 1;
        %         delete(self.b(3));
        %         self.b(3) = uicontrol('Style','pushbutton','String','Back', ...
        %             'Position', [110 150 100 50],'Callback', @self.eStop);
        %     end
        % end

        function jTRAJICwelding(self)
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

            for i = 1:size(startPositions, 1)
                whereRobot = self.titan.model.getpos();
                startTransform = transl(startPositions(i, :)) * trotx(pi);
                startJointAngles = self.titan.model.ikcon(startTransform);
                qmatrix = jtraj(whereRobot, startJointAngles, numSteps);

                for step = 1:numSteps
                    self.titan.model.animate(qmatrix(step, :));
                    pause(0.05);
                end

                tWeld = eye(4) * transl(weldPosition(i, :)) * trotx(pi);
                qWeld = self.titan.model.ikcon(tWeld);
                qWeldMatrix = jtraj(startJointAngles, qWeld, numSteps);

                for step = 1:numSteps
                    self.titan.model.animate(qWeldMatrix(step, :));
                    set(hflange{i}, 'Vertices', self.trVertices(:, 1:3));
                    drawnow
                    pause(0.05);
                end

                pause(5);
                tEnd = eye(4) * transl(endPositions(i, :)) * trotx(pi);
                qEnd = self.titan.model.ikcon(tEnd);
                qEndMatrix = jtraj(qWeld, qEnd, numSteps);

                for step = 1:numSteps
                    self.titan.model.animate(qEndMatrix(step, :));
                    whereEndEffector = self.titan.model.fkine(qEndMatrix(step, :));
                    self.trVertices = [vertices, ones(size(vertices, 1), 1)] * whereEndEffector.T';

                    set(hflange{i}, 'Vertices',self.trVertices(:, 1:3));
                    drawnow();
                    pause(0.05);
                end
                delete(hflange{i});
                PlaceObject('flange.ply', endPositions(i, :));
            end
        end

        function rmrc5(self)
            % self.elipsoidOnRobotTitan();
            % self.elipsoidOnRobotUR3e();
            % Allocate array data
            qMatrix = zeros(self.total_steps,7);                                             % Array for UR3e joint angles
            qMatrix2 = zeros(self.total_steps,6);                                            % Array for Kuka Titan joint angles
            qdot = zeros(self.total_steps,7);                                                % Array for UR3e joint velocities
            qdot2 = zeros(self.total_steps,6);
            theta = zeros(3,self.total_steps);                                               % Array for UR3e roll-pitch-yaw angles
            theta2 = zeros(3,self.total_steps);                                              % Array for Kuka Titan roll-pitch-yaw angles
            self.x_1 = zeros(3,self.total_steps);                                                   % Array for UR3e x-y-z trajectory
            self.x_2 = zeros(3,self.total_steps);                                                  % Array for Kuka Titan x-y-z trajectory

            %% UR3e movement with RMRC (Circular Trajectory)
            radius = 0.15;                  % Radius of the circle
            center = [2; 2; 1.75];        % Center of the circle in (x, y, z) coordinates, 0.5m away in Z
            theta(1,:) = 0;              % Roll angle (fixed)
            theta(2,:) = 5*pi/9;           % Pitch angle (fixed)
            theta(3,:) = pi;              % Yaw angle (fixed)

            s1 = lspb(0, 2*pi, self.steps); % Define angular positions over time for one full circle
            for i = 1:self.steps
                self.x_1(1,i) = center(1) + radius * cos(s1(i)); % x-coordinate for circular path
                self.x_1(2,i) = center(2);                        % y-coordinate (fixed at center)
                self.x_1(3,i) = center(3) + radius * sin(s1(i)); % z-coordinate for circular path
            end

            % Concatenate to complete the circular trajectory by mirroring in reverse
            self.x_1 = [self.x_1, self.x_1(:, end:-1:1)];  % Extend trajectory to move forward and then reverse

            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) self.x_1(:,1); zeros(1,3) 1];  % Initial transformation
            q0 = zeros(1, 7); % Initial guess for joint angles
            qMatrix(1, :) = self.ur3e.model.ikcon(T, q0); % Solve joint angles for first waypoint

            % Check size of x to confirm number of trajectory points
            disp(size(self.x_1)); % Display size of x for debugging

            %% RMRC loop for UR3e following the circular trajectory
            for i = 1:self.steps - 1
                T = self.ur3e.model.fkine(qMatrix(i,:)).T;                                   % Get forward transformation at current joint state
                deltaX = self.x_1(:,i+1) - T(1:3,4);                                           % Position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Desired rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/self.deltaT)*(Rd - Ra);                                            % Rotation matrix error
                self.S = Rdot*Ra';                                                           % Skew symmetric matrix
                linear_velocity = (1/self.deltaT)*deltaX;
                angular_velocity = [self.S(3,2); self.S(1,3); self.S(2,1)];                            % Angular velocity
                xdot = self.W*[linear_velocity; angular_velocity];                           % End-effector velocity
                J = self.ur3e.model.jacob0(qMatrix(i,:));                                    % Jacobian at current state

                % Damped Least Squares (DLS) for low manipulability
                m = sqrt(det(J*J'));
                if m < self.epsilon
                    lambda = (1 - m/self.epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(7))*J';                                   % DLS inverse Jacobian
                qdot(i,:) = (invJ*xdot)';                                               % Joint velocities
                qMatrix(i+1,:) = qMatrix(i,:) + self.deltaT*qdot(i,:);
            end
            % Kuka movement with RMRC
            s2 = lspb(0,1,self.steps);                                                        % Trapezoidal trajectory scalar
            for i = 1:self.steps
                % First segment
                self.x2(:, i) = [(1 - s2(i)) * 1.0 + s2(i) * 2;
                    (1 - s2(i)) * 3 + s2(i) * 2;
                    (1 - s2(i)) * 1.3 + s2(i) * 1.75];
                theta2(:, i) = [0; 5 * pi / 9; 0];
            end

            for i = 1:self.steps
                % Second segment
                self.x2(:, self.steps + i) = [(1 - s2(i)) * 2 + s2(i) * -0.375;
                    (1 - s2(i)) * 2 + s2(i) * 2;
                    (1 - s2(i)) * 1.75 + s2(i) * 0.9];
                theta2(:, self.steps + i) = [0; 5 * pi / 9; 0];
            end

            for i = 1:self.steps
                % Third segment
                self.x2(:, 2 * self.steps + i) = [(1 - s2(i)) * -0.375 + s2(i) * 1.4;
                    2;
                    (1 - s2(i)) * 0.9 + s2(i) * 1.3];
                theta2(:, 2 * self.steps + i) = [0; 5 * pi / 9; 0];
            end

            T2 = [rpy2r(theta2(1,1),theta2(2,1),theta2(3,1)) self.x2(:,1); zeros(1,3) 1];    % Transformation of first point
            q1 = zeros(1,6);                                                            % Initial guess for joint angles
            qMatrix2(1,:) = self.titan.model.ikcon(T2,q1);                                   % Solve joint angles to achieve first waypoint

            % RMRC loop for Kuka Titan
            for i = 1:self.total_steps-1
                T2 = self.titan.model.fkine(qMatrix2(i,:)).T;                                % Get forward transformation at current joint state
                deltaX2 = self.x_2(:,i+1) - T2(1:3,4);                                        % Position error from next waypoint
                Rd2 = rpy2r(theta2(1,i+1),theta2(2,i+1),theta2(3,i+1));                 % Desired rotation matrix
                Ra2 = T2(1:3,1:3);                                                      % Current end-effector rotation matrix
                Rdot2 = (1/self.deltaT)*(Rd2 - Ra2);                                         % Rotation matrix error
                S2 = Rdot2*Ra2';                                                        % Skew symmetric matrix
                linear_velocity2 = (1/self.deltaT)*deltaX2;
                angular_velocity2 = [S2(3,2); S2(1,3); S2(2,1)];                        % Angular velocity
                xdot2 = self.W*[linear_velocity2; angular_velocity2];                        % End-effector velocity
                J2 = self.titan.model.jacob0(qMatrix2(i,:));                                 % Jacobian at current state

                % Damped Least Squares (DLS) for low manipulability
                m2 = sqrt(det(J2*J2'));
                if m2 < self.epsilon
                    lambda = (1 - m2/self.epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ2 = inv(J2'*J2 + lambda *eye(6))*J2';                               % DLS inverse Jacobian
                qdot2(i,:) = (invJ2*xdot2)';                                            % Joint velocities
                qMatrix2(i+1,:) = qMatrix2(i,:) + self.deltaT*qdot2(i,:);                    % Update joint states
            end

            % Plot the UR3e movement
            hold on;
            for i = 1:self.total_steps
                self.ur3e.model.animate(qMatrix(i,:));                                       % Animate UR3e robot
                self.titan.model.animate(qMatrix2(i,:));                                     % Animate Kuka Titan robot
                drawnow();
            end
        end

        function updateJoints(self, source, ~)                              % issue here is that source changes the number
            sliderValue1 = get(source, 'Value');                            % for both of them so they both dance
            sliderValue2 = get(source, 'Value');
            % disp(source);
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
            name = source.String;
            number = str2num(['uint8(',name(2),')']);
            if number == 1
                switch name(1)
                    case 'x'
                        self.x1(1) = sliderValue1;
                    case 'y'
                        self.x1(2) = sliderValue1;
                    case 'z'
                        self.x1(3) = sliderValue1;
                end
            elseif number == 2
                switch name(1)
                    case 'x'
                        self.x2(1) = sliderValue1;
                    case 'y'
                        self.x2(2) = sliderValue1;
                    case 'z'
                        self.x2(3) = sliderValue1;
                end
            end
            for i = 1:100
                delete(self.path(i));
                t = i * 0.01;
                x = self.x1(1) * (1 - t) + self.x2(1) * t;
                y = self.x1(2) * (1 - t) + self.x2(2) * t;
                z = self.x1(3) * (1 - t) + self.x2(3) * t;
                v = [x; y; z];
                d = (v-self.c)'*diag(self.r.^-2)*(v-self.c);
                if d < 1
                    v = self.c + (v-self.c) * d^-0.5;
                end
                self.path(i) = plot3(v(1),v(2),v(3),'r*');
            end
            % path(1) = plot3(self.x1(1), self.x1(2), self.x1(3), 'r*');
            % path(100) = plot3(self.x2(1), self.x2(2), self.x2(3), 'r*');
        end

        function eStop(self, source, ~)
            disp("E has been stopped");
            self.stopped = true;
            delete(source);
            delete([self.b(1); self.b(2); self.b(3); self.b(4); self.s(1); self.s(2); ...
                self.s(3); self.s(4); self.s(5); self.s(6); self.s(7); self.s(8); ...
                self.s(9); self.s(10); self.s(11); self.s(12); self.s(13)]);
            for i = 1:100
                delete(self.path(i));
            end

            self.b(1) = uicontrol('Style','pushbutton','String','Free Control', ...
                'Position', [100 150 100 50],'Callback', @self.freeControl);
            self.b(2) = uicontrol('Style','pushbutton','String','Sequence', ...
                'Position', [100 200 100 50],'Callback', @self.sequence);
            self.b(3) = uicontrol('Style','pushbutton','String','Path Placement', ...
                'Position', [100 100 100 40],'Callback', @self.pathPlacement);
            self.b(4) = uicontrol('Style','pushbutton','String','Controller', ...
                'Position', [100 50 100 40],'Callback', @self.controller);
        end

        function defineObstacle(self, obsX, obsY, obsZ, a)
            %% make the cube
            % [Y,Z] = meshgrid(-0.5:0.05:0.5,-0.5:0.05:0.5);
            [Y,Z] = meshgrid(-a:0.01:a,-a:0.01:a);
            sizeMat = size(Y);
            X = repmat(a,sizeMat(1),sizeMat(2));
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
            set(cube_h, 'Visible', 'off');
        end

        function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)
            algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                + ((points(:,3)-centerPoint(3))/radii(3)).^2;
        end

        function elipsoidOnRobotUR3e(self, obsX, obsY, obsZ, a)
            [Y,Z] = meshgrid(-a:0.01:a,-a:0.01:a);
            sizeMat = size(Y);
            X = repmat(a,sizeMat(1),sizeMat(2));
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
            set(cube_h, 'Visible', 'off');
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
            q = self.ur3e.model.getpos();
            tr = zeros(4,4,self.ur3e.model.n+1);
            tr(:,:,1) = self.ur3e.model.base;
            L = self.ur3e.model.links;
            for i = 1 : self.ur3e.model.n
                tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end
            for i = 2: 6
                warning off;
                cubePointsAndOnes = (inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]')';
                updatedCubePoints = cubePointsAndOnes(:,1:3);
                algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoints(i,:), radii(i, :));
                pointsInside = find(algebraicDist < 1);
                disp(['There are ', num2str(size(pointsInside,1)),' points inside joint ',num2str(i),' ellipsoid']);
                warning on;
            end
        end

        function elipsoidOnRobotTitan(self) %fix center and radii
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
            q = self.titan.model.getpos();
            tr = zeros(4,4,self.titan.model.n+1);
            tr(:,:,1) = self.titan.model.base;
            L = self.titan.model.links;
            for i = 1 : self.titan.model.n
                tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end
            for i = 2: 6
                warning off;
                cubePointsAndOnes = (inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]')';
                updatedCubePoints = cubePointsAndOnes(:,1:3);
                algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoints(i,:), radii(i, :));
                pointsInside = find(algebraicDist < 1);
                disp(['There are ', num2str(size(pointsInside,1)),' points inside joint ',num2str(i),' ellipsoid']);
                warning on;
            end
        end
    end
end
