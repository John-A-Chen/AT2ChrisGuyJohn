clear all;
close all;
clc

robot = UR3;

% New values for the ellipsoid
centerPoint = [0,0,0.075];
radii = [0.1,0.1,0.15];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );

ellipsoidHandles = [];  % Initialize an array to store ellipsoid graphics handles

for i = 1:4
    % Assign points to the robot model
    robot.model.points{i} = [X(:), Y(:), Z(:)];
    robot.model.faces{i} = delaunay(robot.model.points{i});
    
    % Plot each ellipsoid and capture its handle
    ellipsoidHandles(i) = surf(X, Y, Z);  % Store the graphics handle for each ellipsoid
    hold on;
end

% Set the transparency of only the ellipsoids
for i = 1:4
    alpha(ellipsoidHandles(i), 0.1);  % Adjust the transparency level (0.0 to 1.0)
end

robot.model.plot3d([0,0,0,0,0,0]);
robot.model.teach;
