classdef A2 < handle
    properties
        totalTime 
        S = 5; 
        ur3e;
        q_ur3e;
        titan;
        q_titan;
        stoped = false;

        s1;
        s2;
        s3;
        s4;
        s5;
        s6;
        b1;
        b2;
        b3;
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
            % self.ur3e = UR3e(transl(0, 2, 0.05));
            % self.q_ur3e = zeros(1,6);
            self.titan = KukaTitan(transl(0, -2, 0.05));
            self.q_titan = zeros(1,6);
            self.setupEnvironment();
            elapsedTime = toc(self.totalTime); 
            disp(['Total elapsed time: ', num2str(elapsedTime), ' seconds']);
            self.DLS();

            % self.b1 = uicontrol('Style','pushbutton','String','Free Control','Position', [0 70 100 10],'Callback', @self.freeControl);
            % self.b2 = uicontrol('Style','pushbutton','String','Sequence','Position', [0 60 100 10],'Callback', @self.sequence);
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
        end

        function freeControl(self, source, ~)
            delete(source);
            delete(self.b2)
            sliderProperties = struct('Style', 'slider', 'Value', 0, 'SliderStep', [0.01 0.2], 'Callback', @self.updateJoints);
            self.s1 = uicontrol(sliderProperties ,  'Position', [0 60 200 10], 'String', 'q1', 'Min', -150 ,  'Max', 150, "Value", self.q_titan(1) * 180/pi);
            self.s2 = uicontrol(sliderProperties ,  'Position', [0 50 200 10], 'String', 'q2', 'Min', -40 ,  'Max', 107.5, "Value", self.q_titan(2) * 180/pi);
            self.s3 = uicontrol(sliderProperties ,  'Position', [0 40 200 10], 'String', 'q3', 'Min', -200 ,  'Max', 55, "Value", self.q_titan(3) * 180/pi);
            self.s4 = uicontrol(sliderProperties,   'Position', [0 30 200 10], 'String', 'q4','Min', -350,'Max', 350, "Value", self.q_titan(4) * 180/pi);
            self.s5 = uicontrol(sliderProperties,   'Position', [0 20 200 10], 'String', 'q5','Min', -118,'Max', 118, "Value", self.q_titan(5) * 180/pi);
            self.s6 = uicontrol(sliderProperties,   'Position', [0 10 200 10], 'String', 'q6','Min', -350,'Max', 350, "Value", self.q_titan(6) * 180/pi);
            
            self.b1 = uicontrol('Style','pushbutton','String','E-Stop','Position', [0 70 50 10],'Callback', @self.eStop);
        end

        function sequence(self, source, ~)
            delete(source);
            delete(self.b1);
            self.b1 = uicontrol('Style','pushbutton','String','E-Stop','Position', [0 70 50 10],'Callback', @self.eStop);
            
        end

        function DLS(self)
            t = 10;                    % Total time in seconds
            steps = 200;                % No. of steps
            deltaT = t/steps;           % Discrete time step
            deltaTheta = 4*pi/steps;    % Small angle change
            qMatrix = zeros(steps,6);   % Assign memory for joint angles
            x = zeros(3,steps);         % Assign memory for trajectory
            m = zeros(1,steps);         % For recording measure of manipulability
            errorValue = zeros(3,steps);% For recording velocity error
            lambda = 0;
            lambdaMax = 0.05;
            epsilon = 0.5;

            for i = 1:steps
                x(:,i) = [1.5*cos(deltaTheta*i) + 0.45*cos(deltaTheta*i)
                          1.5*sin(deltaTheta*i) + 0.45*cos(deltaTheta*i)
                          0.5];
            end

            qMatrix(1,:) = self.titan.model.ikcon(transl(x(:,1)));
            self.titan.model.animate(qMatrix(1,:));

            for i = 1:steps-1
                T = self.titan.model.fkine(qMatrix(i,:)).T;                    % End-effector transform at current joint state
                xdot = (x(:,i+1)-T(1:3,4));                  % Velocity to reach next waypoint
                J = self.titan.model.jacob0(qMatrix(i,:));                     % Get Jacobian at current state (use jacob0)
                J = J(1:3,:);               % Take only first 3 rows
                m(:,i) = sqrt(det(J*J'));   % Measure of Manipulability
                if m(:,i) > epsilon
                    lambda = 0;
                else
                    lambda = (1 - (m(:,i)/epsilon)^2) * lambdaMax;
                end
                qdot = J'*inv(J*J' + lambda * eye(3))*xdot;                  % Solve the RMRC equation
                errorValue(:,i) = xdot - J*qdot; % Velocity error
                qMatrix(i+1,:)= qMatrix(i,:) + (qdot)';         % Update the joint state
                self.titan.model.plot(qMatrix(i+1,:));
            end
        end

        function updateJoints(self, source, ~)
            sliderValue = get(source, 'Value');
            number = source.String;
            number = str2num(['uint8(',number(2),')']);
            q = self.q_titan;
            q(number) = sliderValue * pi/180;
            self.q_titan = q;
            self.titan.model.animate(q);
        end

        function eStop(self, source, ~)
            disp("E has been stopped");

            delete(source);
            delete(self.s1);
            delete(self.s2);
            delete(self.s3);
            delete(self.s4);
            delete(self.s5);
            delete(self.s6);
            
            self.b1 = uicontrol('Style','pushbutton','String','Free Control','Position', [0 70 100 10],'Callback', @self.freeControl);
            self.b2 = uicontrol('Style','pushbutton','String','Sequence','Position', [0 60 100 10],'Callback', @self.sequence);
        end
    end
end
