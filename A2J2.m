classdef A2J2 < handle
    properties
        totalTime
        Size = 5;
        ur3e;
        q_ur3e;
        titan;
        q_titan;
        stopped = false;
        a = 0.1;
        s = gobjects(1,13);
        b = gobjects(1,4);
        t = 1;                                                                      % Total time for one direction (s)
        deltaT = 0.02;                                                              % Control frequency
        steps = 1/0.2;                                                           % Number of steps for one direction
        total_steps = 2*1/0.2;                                                      % Total steps for forward and backward
        S = 5;
        delta = 2*pi/(1/0.2);                                                         % Small angle change
        epsilon = 0.1;                                                              % Threshold for manipulability/Damped Least Squares
        W = diag([1 1 1 0.1 0.1 0.1]);                                              % Weighting matrix for the velocity vector
        obsX = 2;
        obsY = 0;
        obsZ = 1.5;
        cubePoints;
        algebraicDist;
    end

    methods
        function self = A2J2()
            tic;
            self.totalTime = tic;
            clf;
            axis equal;
            hold on;
            view(3);
            axis([-self.Size, self.Size, -self.Size, self.Size, 0, self.Size]);
            xlim([-self.Size, self.Size]);
            ylim([-self.Size, self.Size]);
            zlim([0, self.Size]);
            % self.a = arduino;
            self.ur3e = UR3e(transl(2.5, 1.75, 0.925));
            self.q_ur3e = zeros(1,6);
            self.titan = KukaTitan(transl(-0.75,0.5,0.05));
            self.q_titan = zeros(1,6);
            self.setupEnvironment();
            elapsedTime = toc(self.totalTime);
            disp(['Total elapsed time: ', num2str(elapsedTime), ' seconds']);
            self.startUI();
            self.defineObstacle();
            % self.elipsoidOnRobotUR3e(3,1,1,0.2);
        end

        function setupEnvironment(self)
            surf([-self.Size, -self.Size; self.Size, self.Size], ...
                [-self.Size, self.Size; -self.Size, self.Size], [0, 0; 0, 0], ...
                'CData', imread('concrete.jpg'), 'FaceColor', 'texturemap');
            surf([self.Size, self.Size; self.Size, self.Size], ...
                [-self.Size, self.Size; -self.Size, self.Size], [self.Size, self.Size; 0, 0], ...
                'CData', imread('IMG_7414.jpg'), 'FaceColor', 'texturemap');
            surf([-self.Size, self.Size; -self.Size, self.Size], ...
                [self.Size, self.Size; self.Size, self.Size], [self.Size, self.Size; 0, 0], ...
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
                'Position', [1200 20 1 1], 'String', 'uq7', ...
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
                self.s(9); self.s(10); self.s(11); self.s(12); self.s(13)]);

            self.b(3) = uicontrol('Style','pushbutton','String','E-Stop', ...
                'Position', [110 150 100 50],'Callback', @self.eStop);

            self.b(2) = uicontrol('Style','pushbutton','String','Free Control', ...
                'Position', [110 200 100 50],'Callback', @self.freeControl);

            self.rmrc5();
        end

        function rmrc5(self)
            self.elipsoidOnTitan();
            self.elipsoidOnUR3e();
            % Allocate array data
            qMatrix = zeros(self.total_steps,7);                                             % Array for UR3e joint angles
            qMatrix2 = zeros(self.total_steps,6);                                            % Array for Kuka Titan joint angles
            qdot = zeros(self.total_steps,7);                                                % Array for UR3e joint velocities
            qdot2 = zeros(self.total_steps,6);
            theta = zeros(3,self.total_steps);                                               % Array for UR3e roll-pitch-yaw angles
            theta2 = zeros(3,self.total_steps);                                              % Array for Kuka Titan roll-pitch-yaw angles
            x = zeros(3,self.total_steps);                                                   % Array for UR3e x-y-z trajectory
            x2 = zeros(3,self.total_steps);                                                  % Array for Kuka Titan x-y-z trajectory

            % UR3e movement with RMRC
            s1 = lspb(0,1,self.steps);                                                       % Trapezoidal trajectory scalar
            for i=1:self.steps
                x(1,i) = 1.25;                                                             % Points in x (fixed)
                x(2,i) = (1-s1(i))*1.25 + s1(i)*3.25;                                        % Points in y (move forward)
                x(3,i) = 1.5;                                                             % Points in z (fixed)
                theta(1,i) = 0;                                                         % Roll angle
                theta(2,i) = 5*pi/9;                                                    % Pitch angle
                theta(3,i) = 0;                                                         % Yaw angle
            end
            for i=1:self.steps
                x(1,self.steps+i) = 1.25;                                                       % Points in x (fixed)
                x(2,self.steps+i) = (1-s1(i))*3.25 + s1(i)*1.25;                                  % Points in y (move backward)
                x(3,self.steps+i) = 1.5;                                                       % Points in z (fixed)
                theta(1,self.steps+i) = 0;                                                   % Roll angle
                theta(2,self.steps+i) = 5*pi/9;                                              % Pitch angle
                theta(3,self.steps+i) = 0;                                                   % Yaw angle
            end

            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1); zeros(1,3) 1];         % Transformation of first point
            q0 = zeros(1,7);                                                            % Initial guess for joint angles
            qMatrix(1,:) = self.ur3e.model.ikcon(T,q0);                                      % Solve joint angles to achieve first waypoint

            % RMRC loop for UR3e
            for i = 1:self.total_steps-1
                T = self.ur3e.model.fkine(qMatrix(i,:)).T;                                   % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                           % Position error from next waypoint
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
                qMatrix(i+1,:) = qMatrix(i,:) + self.deltaT*qdot(i,:);                       % Update joint states
            end

            % Kuka movement with RMRC
            s2 = lspb(0,1,self.steps);                                                        % Trapezoidal trajectory scalar
            for i=1:self.steps
                x2(1,i) = 1.25;                                                            % Points in x (fixed)
                x2(2,i) = (1-s2(i))*1.25 + s2(i)*3.25;                                 % Points in y (move forward)
                x2(3,i) = 1.5;                                                          % Points in z (fixed)
                theta2(1,i) = 0;                                                        % Roll angle
                theta2(2,i) = 5*pi/9;                                                   % Pitch angle
                theta2(3,i) = 0;                                                        % Yaw angle
            end
            for i=1:self.steps
                x2(1,self.steps+i) = 1.25;                                                      % Points in x (fixed)
                x2(2,self.steps+i) = (1-s2(i))*3.25 + s2(i)*1.25;                           % Points in y (move backward)
                x2(3,self.steps+i) = 1.5;                                                    % Points in z (fixed)
                theta2(1,self.steps+i) = 0;                                                  % Roll angle
                theta2(2,self.steps+i) = 5*pi/9;                                             % Pitch angle
                theta2(3,self.steps+i) = 0;                                                  % Yaw angle
            end

            T2 = [rpy2r(theta2(1,1),theta2(2,1),theta2(3,1)) x2(:,1); zeros(1,3) 1];    % Transformation of first point
            q1 = zeros(1,6);                                                            % Initial guess for joint angles
            qMatrix2(1,:) = self.titan.model.ikcon(T2,q1);                                   % Solve joint angles to achieve first waypoint

            % RMRC loop for Kuka Titan
            for i = 1:self.total_steps-1
                T2 = self.titan.model.fkine(qMatrix2(i,:)).T;                                % Get forward transformation at current joint state
                deltaX2 = x2(:,i+1) - T2(1:3,4);                                        % Position error from next waypoint
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
        end

        function eStop(self, source, ~)
            % disp(so)
            disp("E has been stopped");
            self.stopped = true;
            delete(source);
            delete([self.b(1); self.b(2); self.b(3); self.b(4); self.s(1); self.s(2); ...
                self.s(3); self.s(4); self.s(5); self.s(6); self.s(7); self.s(8); ...
                self.s(9); self.s(10); self.s(11); self.s(12); self.s(13)]);

            self.b(1) = uicontrol('Style','pushbutton','String','Free Control', ...
                'Position', [100 150 100 50],'Callback', @self.freeControl);
            self.b(2) = uicontrol('Style','pushbutton','String','Sequence', ...
                'Position', [100 200 100 50],'Callback', @self.sequence);
        end
        function defineObstacle(self)
            [Y,Z] = meshgrid(-self.a:0.01:self.a,-self.a:0.01:self.a);
            sizeMat = size(Y);
            X = repmat(self.a,sizeMat(1),sizeMat(2));
            oneSideOfCube_h = surf(X,Y,Z);
            self.cubePoints = [X(:),Y(:),Z(:)];
            self.cubePoints = [ self.cubePoints ...
                ; self.cubePoints * rotz(pi/2)...
                ; self.cubePoints * rotz(pi) ...
                ; self.cubePoints * rotz(3*pi/2) ...
                ; self.cubePoints * roty(pi/2) ...
                ; self.cubePoints * roty(-pi/2)];
            % cubeAtOigin_h = plot3(self.cubePoints(:,1),self.cubePoints(:,2),self.cubePoints(:,3),'r.');
            self.cubePoints = self.cubePoints + repmat([self.obsX,self.obsY,self.obsZ],size(self.cubePoints,1),1);
            cube_h = plot3(self.cubePoints(:,1),self.cubePoints(:,2),self.cubePoints(:,3),'b.');
            set(cube_h, 'Visible', 'off');
        end

        function algebraicDist = GetAlgebraicDist(self, points, centerPoint, radii)
            algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                + ((points(:,3)-centerPoint(3))/radii(3)).^2;
        end

        function elipsoidOnUR3e(self)
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
                cubePointsAndOnes = (inv(tr(:,:,i)) * [self.cubePoints,ones(size(self.cubePoints,1),1)]')';
                updatedCubePoints = cubePointsAndOnes(:,1:3);
                self.algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoints(i,:), radii(i, :));
                pointsInside = find(self.algebraicDist < 1);
                disp(['There are ', num2str(size(pointsInside,1)),' points inside joint ',num2str(i),' ellipsoid']);
                warning on;
            end
        end

        function elipsoidOnTitan(self)
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
                cubePointsAndOnes = (inv(tr(:,:,i)) * [self.cubePoints,ones(size(self.cubePoints,1),1)]')';
                updatedCubePoints = cubePointsAndOnes(:,1:3);
                self.algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoints(i,:), radii(i, :));
                pointsInside = find(self.algebraicDist < 1);
                disp(['There are ', num2str(size(pointsInside,1)),' points inside joint ',num2str(i),' ellipsoid']);
                warning on;
            end
        end
    end
end
