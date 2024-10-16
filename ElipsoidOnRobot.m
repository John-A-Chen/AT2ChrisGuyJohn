clear all;
close all
clc

robot = UR3;

% L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
% L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
% L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])        
% robot = SerialLink([L1 L2 L3],'name','myRobot');  

%robot = UR3

% New values for the ellipsoid (guessed these, need proper model to work out correctly)
centerPoint = [0,0,0];
radii = [0.2,0.05,0.05];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
for i = 1:4
    robot.model.points{i} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{i} = delaunay(robot.model.points{i});    
    warning on;
end

robot.model.plot3d([0,0,0,0,0,0]);
axis equal
camlight

robot.model.teach;