classdef KukaTitan < RobotBaseClass
    properties(Access = public)
        plyFileNameStem = 'KUKATitan';
    end

    methods
        %% Define robot Function
        function self = KukaTitan(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            self.PlotAndColourRobot();
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
        end
    end
end
