classdef LinearNachiMZ04 < RobotBaseClass
    %% Linear MZ04-01-CFD robot model
    % WARNING: This model has been created by john haha
    % No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. 
    % Please assume that this matches the real robot!
    properties(Access = public)
        plyFileNameStem = 'robotarm';
    end
    methods
        %% Define robot Function
        function self = LinearNachiMZ04(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr;
            self.PlotAndColourRobot();
            axis([-1, 1, -1, 1, 0, 1.5]);
            % hold on
        end
        %% CreateModel
        function CreateModel(self)
            L(1) = Link([pi     0       0       0    1]);    
            L(2) = Link('d' ,0.09, 'a', 0.00, 'alpha', 0,     'offset', pi/2,  'qlim', [-180, 180] * pi/180);
            L(3) = Link('d', 0.25, 'a', 0.00, 'alpha', -pi/2, 'offset', 0,     'qlim', [-170, 170] * pi/180);
            L(4) = Link('d', 0.00, 'a', 0.20, 'alpha', 0,     'offset', -pi/2, 'qlim', [-090, 145] * pi/180);
            L(5) = Link('d', 0.00, 'a', 0.02, 'alpha', -pi/2, 'offset', 0,     'qlim', [-180, 070] * pi/180);
            L(6) = Link('d', 0.21, 'a', 0.00, 'alpha', pi/2,  'offset', 0,     'qlim', [-190, 190] * pi/180);
            L(7) = Link('d', 0.00, 'a', 0.00, 'alpha', -pi/2, 'offset', 0,     'qlim', [-120, 120] * pi/180);
            L(8) = Link('d', 0.06, 'a', 0.00, 'alpha', 0,     'offset', 0,     'qlim', [-360, 360] * pi/180);
            self.model = SerialLink(L, 'name', self.name);
            axis equal;
        end
    end
end