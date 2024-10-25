clear all;
close all;
clc;

robot = UR3;  % Define the robot model

% Define the center points and radii for the ellipsoids for each link
centerPoints = [
    0, 0, 0;   % Link 1
    0, 0, 0.07;   % Link 2
    0, 0, 0.07;   % Link 3
    0, 0, 0;   % Link 4
    0, 0, 0;   % Link 5
    0, 0, 0;   % Link 6
];

radii = [
    0, 0, 0; % Link 1
    0.14, 0.1, 0.14; % Link 2
    0.2, 0.1, 0.14; % Link 3
    0.1, 0.1, 0.1; % Link 4
    0.1, 0.1, 0.1; % Link 5
    0.1, 0.1, 0.1; % Link 6
];



% Loop through each link to create and attach ellipsoids
for i = 2:5
    [X, Y, Z] = ellipsoid(centerPoints(i,1), centerPoints(i,2), centerPoints(i,3), radii(i,1), radii(i,2), radii(i,3), 6); % Create the ellipsoid for the current link
    
    robot.model.points{i} = [X(:),Y(:),Z(:)]; % defines points for elipsoid triangulation (comment out to encapsulate links with a bug)

    robot.model.faces{i} = delaunay(robot.model.points{i}); % creates elipsoid faces
   

    %trisurf(delaunay(X,Y,Z), X, Y, Z, 'FaceAlpha', 0.5);  % Draw the ellipsoid with some transparency
 
    hold on;
end


% Plot the robot
robot.model.plot3d(zeros(1,6));
axis equal
camlight

% Activate the teach interface to move the robot
robot.model.teach;