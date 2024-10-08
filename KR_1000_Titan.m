%%KR 1000 titan
% clear;
% 
% L(1) = Link('d', 1.1, 'a', 0.6, 'alpha', -pi/2, 'qlim', [-150, 150] * pi/180);
% L(2) = Link('d', 0, 'a', 1.465, 'alpha', 0, 'qlim', [-130, 17.5] * pi/180);
% L(3) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-110 + 90, 145 + 90] * pi/180);
% L(4) = Link('d', 1.6, 'a', 0, 'alpha', -pi/2, 'qlim', [-350, 350] * pi/180);
% L(5) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-118, 118] * pi/180);
% L(6) = Link('d', 0.372, 'a', 0, 'alpha', 0, 'qlim', [-350, 350] * pi/180);
% 
% titan = SerialLink(L);
% titan.teach()

classdef KR_1000_Titan < RobotBaseClass
    %% UR3e Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'KR_1000_Titan';
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
			warning('The DH parameters are correct. But as of July 2023 the ply files for this UR3e model are definitely incorrect, since we are using the UR3 ply files renamed as UR3e. Once replaced remove this warning.')  
            self.PlotAndColourRobot();

            drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d', 1.1, 'a', 0.6, 'alpha', -pi/2, 'qlim', [-150, 150] * pi/180);
            link(2) = Link('d', 0, 'a', 1.465, 'alpha', 0, 'qlim', [-130, 17.5] * pi/180);
            link(3) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-110 + 90, 145 + 90] * pi/180);
            link(4) = Link('d', 1.6, 'a', 0, 'alpha', -pi/2, 'qlim', [-350, 350] * pi/180);
            link(5) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-118, 118] * pi/180);
            link(6) = Link('d', 0.372, 'a', 0, 'alpha', 0, 'qlim', [-350, 350] * pi/180);
             
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
