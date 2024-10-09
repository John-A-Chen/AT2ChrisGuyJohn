classdef A2
    properties
        totalTime                                                        % store the total elapsed time
        r;                                                               % linearUR3e
    end
    methods
        function self = A2()
            tic; % Start the timer
            self.totalTime = tic;
            clf;                                                         % Constructor (this is like main)
            axis equal;
            hold on;
            view(3);
            axis ([-2 2 -2 2 0 2]);
            xlim([-2 2]); ylim([-2 2]); zlim([0 2]);
            self.r = DobotMagician();                  
            self.setupEnvironment();

            disp('arbituary comment!');
            elapsedTime = toc(self.totalTime);                           % total elapsed time
            disp(['Total elapsed time: ', num2str(elapsedTime), ' seconds']);
        end

        function setupEnvironment(~)

            surf([-5,-5;4,4], [-3,3;-3,3], [0,0;0,0], 'CData', ...       % create the floor surface
                imread('concrete.jpg'), 'FaceColor', 'texturemap');

            surf([4,4;4,4], [-3,3;-3,3], [3, 3; 0,0], 'CData', ...       % create the wall surface 1
                imread('IMG_7414.jpg'), 'FaceColor', 'texturemap');

            surf([-5,4;-5,4], [3,3;3,3], [3, 3; 0,0], 'CData', ...       % create the wall surface 2
                imread('IMG_7413.jpg'), 'FaceColor', 'texturemap');
        end
    end
end