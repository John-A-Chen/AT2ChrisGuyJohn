clear all;
close all
clc

%axis([-2 2 -2 2 0 2]);
axis equal
view(3);  % Set the view to 3D
grid on;

robot = UR3e;
%robot.model.base()
 
% New values for the ellipsoid (guessed these, need proper model to work out correctly)
centerPoint = [0,0,0];
radii = [0.2,0.05,0.1];    % change to for loop for different sized elipsoids
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

for i = 1:4
    robot.model.points{i} = [X(:),Y(:),Z(:)];
    % warning off
    robot.model.faces{i} = delaunay(robot.model.points{i});
    % warning on;
end


%%robot.model.plot3d([0,0,0,0,0,0]);
%%robot.model.plot([0,0,0,0,0,0]);
robot.model.plot3d([0,0,0,0,0,0]);
% axis equal
% camlight

robot.model.teach;  