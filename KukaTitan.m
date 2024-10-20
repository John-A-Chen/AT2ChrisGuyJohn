classdef KukaTitan < RobotBaseClass
    properties(Access = public)
        plyFileNameStem = 'Titan2';
    end
    methods
        %% Define robot Function
        function self = KukaTitan(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr;
            self.PlotAndColourRobot();
            axis([-3, 3, -3, 3, 0, 5]);
            % hold on
        end
        %% CreateModel
        function CreateModel(self)
            L(1) = Link('d', 1.100, 'a', 0.600, 'alpha', -pi/2, 'offset', 0.000,'qlim', [-150, 150] * pi/180);
            L(2) = Link('d', 0.000, 'a', 1.465, 'alpha', 0.000, 'offset', -pi/2,'qlim', [-40, 107.5] * pi/180);
            L(3) = Link('d', 0.000, 'a', 0.000, 'alpha', pi/2,  'offset', pi,   'qlim', [-200, 55] * pi/180);
            L(4) = Link('d', 1.200, 'a', 0.000, 'alpha', -pi/2, 'offset', 0.000,'qlim', [-350, 350] * pi/180);
            L(5) = Link('d', 0.000, 'a', 0.000, 'alpha', pi/2,  'offset', 0.000,'qlim', [-118, 118] * pi/180);
            L(6) = Link('d', 0.372, 'a', 0.000, 'alpha', 0.000, 'offset', 0.000,'qlim', [-350, 350] * pi/180);
            self.model = SerialLink(L, 'name', self.name);
            axis equal;
        end
    end
end