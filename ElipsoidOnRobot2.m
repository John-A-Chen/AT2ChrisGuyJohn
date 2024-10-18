clear all;
close all;
clc;

robot = UR3;  % Define the robot model

% Define the center points and radii for the ellipsoids for each link
centerPoints = [
    0, 0, 0;   % Link 1
    0, 0, 0;   % Link 2
    0, 0, 0;   % Link 3
    0, 0, 0;   % Link 4
    0, 0, 0;   % Link 5
    0, 0, 0    % Link 6
];

radii = [
    0.1, 0.1, 0.2; % Link 1
    0.1, 0.1, 0.2; % Link 2
    0.1, 0.1, 0.2; % Link 3
    0.1, 0.1, 0.2; % Link 4
    0.1, 0.1, 0.2; % Link 5
    0.1, 0.1, 0.2  % Link 6
];


% Loop through each link to create and attach ellipsoids
for i = 1:4
    % Create the ellipsoid for the current link
    [X, Y, Z] = ellipsoid(centerPoints(i,1), centerPoints(i,2), centerPoints(i,3), radii(i,1), radii(i,2), radii(i,3));
    
    % Transform the ellipsoid to match the link's position
    ellipsoidPoints = [X(:), Y(:), Z(:)];
    
    % Attach ellipsoid to the robot's link
    trisurf(delaunay(X,Y,Z), X, Y, Z, 'FaceAlpha', 0.1);  % Draw the ellipsoid with some transparency
    
    % Hold on to plot each ellipsoid without clearing the previous ones
    hold on;
end

% Plot the robot
robot.model.plot3d(zeros(1,6));
axis equal
camlight

% Activate the teach interface to move the robot
robot.model.teach;