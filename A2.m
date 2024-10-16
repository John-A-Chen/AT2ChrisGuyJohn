classdef A2
    properties
        totalTime  
        % r;
        % r2;
        S = 5; 
        dobot;
        q_dobot
        titan;
        q_titan;
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
            self.dobot = DobotMagician(transl(0, 2, 0.05));
            self.q_dobot = zeros(1,6);
            self.titan = KukaTitan(transl(0, -2, 0.05));  
            self.q_titan = zeros(1,6);
            self.setupEnvironment();
            disp('Arbitrary comment!');
            elapsedTime = toc(self.totalTime); 
            disp(['Total elapsed time: ', num2str(elapsedTime), ' seconds']);
            self.freeControl();
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

        function freeControl(self)
            self.titan.model.animate(self.q_titan);

            sliderProperties = struct('Style', 'slider', 'Value', 0, 'SliderStep', [0.01 0.2], 'Callback', @self.updateJoints);
            uicontrol(sliderProperties ,  'Position', [0 60 200 10], 'String', 'q1', 'Min', -150 ,  'Max', 150);
            uicontrol(sliderProperties ,  'Position', [0 50 200 10], 'String', 'q2', 'Min', -40 ,  'Max', 107.5);
            uicontrol(sliderProperties ,  'Position', [0 40 200 10], 'String', 'q3', 'Min', -200 ,  'Max', 55);
            uicontrol(sliderProperties,   'Position', [0 30 200 10], 'String', 'q4','Min', -350,'Max', 350);
            uicontrol(sliderProperties,   'Position', [0 20 200 10], 'String', 'q5','Min', -118,'Max', 118);
            uicontrol(sliderProperties,   'Position', [0 10 200 10], 'String', 'q6','Min', -350,'Max', 350);
        end

        function updateJoints(self,source, ~)
            sliderValue = get(source, 'Value');
            number = source.String;
            number = str2num(['uint8(',number(2),')']);
            self.q_titan(number) = sliderValue * pi/180;
            self.titan.model.animate(self.q_titan);
        end
    end
end
