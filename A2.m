classdef A2
    properties
        totalTime  
        r;
        r2;
        S = 5;  
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
            self.r = DobotMagician(transl(0, 2, 0.05));
            self.r2 = SigmaKukaTitan(transl(0, -2, 0.05));  
            self.setupEnvironment();
            disp('Arbitrary comment!');
            elapsedTime = toc(self.totalTime); 
            disp(['Total elapsed time: ', num2str(elapsedTime), ' seconds']);
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
    end
end
