classdef SigmaKukaTitan < RobotBaseClass
    properties(Access = public)
        plyFileNameStem = 'Titan1';
    end

    methods
        %% Define robot Function
        function self = SigmaKukaTitan(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr;
            % self.model.plot(zeros(6));
            self.PlotAndColourRobot();
            axis([-5, 5, -5, 5, -1, 5]);
            % hold on
            % PlaceObject('KUKATitanLink0.PLY');
            % PlaceObject('KUKATitanLink1.PLY');
        end
        %% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0,   'a',0,   'alpha',0,   'offset',0,   'qlim',deg2rad([-150,150]));
            link(2) = Link('d',0,   'a',0,   'alpha',0,   'offset',0,   'qlim',deg2rad([-130,17.5]));
            link(3) = Link('d',0,   'a',0,   'alpha',0,   'offset',0,   'qlim',deg2rad([-110,145]));
            link(4) = Link('d',0,   'a',0,   'alpha',0,   'offset',0,   'qlim',deg2rad([-350,350]));
            link(5) = Link('d',0,   'a',0,   'alpha',0,   'offset',0,   'qlim',deg2rad([-118,118]));
            link(6) = Link('d',0,   'a',0,   'alpha',0,   'offset',0,   'qlim',deg2rad([-350,350]));
            self.model = SerialLink(link, 'name', self.name);
            axis equal;
        end
    end
end
