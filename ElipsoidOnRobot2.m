clear all;
close all;
clc;

%%  making cube point cloud for obsticle
% One side of the cube
[Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
sizeMat = size(Y);
X = repmat(0.75,sizeMat(1),sizeMat(2));
oneSideOfCube_h = surf(X,Y,Z);

% Combine one surface as a point cloud
cubePoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
cubePoints = [ cubePoints ...
             ; cubePoints * rotz(pi/2)...
             ; cubePoints * rotz(pi) ...
             ; cubePoints * rotz(3*pi/2) ...
             ; cubePoints * roty(pi/2) ...
             ; cubePoints * roty(-pi/2)];         
         
% Plot the cube's point cloud         
cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
cubePoints = cubePoints + repmat([2,0,-0.5],size(cubePoints,1),1);
cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
axis equal

%% robotic aerm with ellipsoids
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
    1, 1, 1; % Link 1
    0.14, 0.1, 0.14; % Link 2
    0.2, 0.1, 0.14; % Link 3
    0.1, 0.1, 0.1; % Link 4
    0.1, 0.1, 0.1; % Link 5
    0.1, 0.1, 0.1; % Link 6
];

% Loop through each link to create and attach ellipsoids
for i = 2:6
    [X, Y, Z] = ellipsoid(centerPoints(i,1), centerPoints(i,2), centerPoints(i,3), radii(i,1), radii(i,2), radii(i,3), 6); % Create the ellipsoid for the current link
    robot.model.points{i} = [X(:),Y(:),Z(:)]; % defines points for elipsoid triangulation (comment out to encapsulate links with a bug)
    robot.model.faces{i} = delaunay(robot.model.points{i}); % creates elipsoid faces
    hold on;
end


% Plot the robot
robot.model.plot3d(zeros(1,6));
axis equal
camlight

% Activate the teach interface to move the robot
robot.model.teach;