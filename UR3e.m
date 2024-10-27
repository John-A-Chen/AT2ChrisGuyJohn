classdef UR3e < RobotBaseClass
    %% UR3e Universal Robot 3kg payload robot model
    properties(Access = public)
        plyFileNameStem = 'UR3e';
    end

    methods
        %% Constructor
        function self = UR3e(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);
                end
            else % All passed in
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end

            self.CreateModel();
            self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
        end

        %% CreateModel
        function CreateModel(self)
            link(1) = Link('d', 0.3037, 'a', 0,        'alpha', pi/2, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(2) = Link('d', 0,      'a', -0.48710, 'alpha', 0,    'qlim', deg2rad([-90 90]), 'offset', -pi/2);
            link(3) = Link('d', 0,      'a', -0.4264,  'alpha', 0,    'qlim', deg2rad([-170 170]), 'offset', 0);
            link(4) = Link('d', 0.2247, 'a', 0,        'alpha', pi/2, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(5) = Link('d', 0.1707, 'a', 0,        'alpha', -pi/2,'qlim', deg2rad([-360,360]), 'offset', 0);
            link(6) = Link('d', 0.1638, 'a', 0,        'alpha', 0,    'qlim', deg2rad([-360,360]), 'offset', 0);
            link(7) = Link('d', 0.330,  'a', 0,        'alpha', 0,    'qlim', deg2rad([-360,360]), 'offset', 0);
            self.model = SerialLink(link,'name',self.name);
        end
    end
end
