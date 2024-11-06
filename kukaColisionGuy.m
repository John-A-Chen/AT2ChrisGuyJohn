close all
clear all
clc

stopped = false;
            axis([-3, 3, -3, 3, 0, 3]);

% 1.1) Set parameters for the simulation
% Define the center points and radii for the ellipsoids for each link
centerPoints = [
    0, 0, 0;   % Link 1
    -0.6, 0, 0;   % Link 2
    -0.6, 0, -0.3;   % Link 3
    0, 0, 0.2;   % Link 4
    0, 0, 0;   % Link 5
    0, 0, 0.5;   % Link 6
    ];

radii = [
    0, 0, 0; % Link 1
    1.25, 0.6, 0.5; % Link 2
    0.8, 0.3, 0.5; % Link 3
    0.5, 0.4, 0.6; % Link 4
    0.2, 0.5, 0.4; % Link 5
    0.2, 0.2, 0.2; % Link 6
    ];


hold on
titan = KukaTitan();                                                          % Load robot model
t = 2;                                                                      % Total time for one direction (s)
deltaT = 0.02;                                                              % Control frequency
steps = t/deltaT;                                                           % Number of steps for one direction
total_steps = 2*steps;                                                      % Total steps for forward and backward
S = 5;

axis(5,5,5)
view(0, 90);


for i = 2:6
    [X, Y, Z] = ellipsoid(centerPoints(i,1), centerPoints(i,2), centerPoints(i,3), radii(i,1), radii(i,2), radii(i,3), 6); % Create the ellipsoid for the current link
    titan.model.points{i} = [X(:),Y(:),Z(:)]; % defines points for elipsoid triangulation (comment out to encapsulate links with a bug)
    titan.model.faces{i} = delaunay(titan.model.points{i}); % creates elipsoid faces
    hold on;
end

%titan.model.plot3d([0 0 0 0 0 0]);
%defineObstacle(0,2, 0, 1.5, 0.2)




%% make the cube
% [Y,Z] = meshgrid(-0.5:0.05:0.5,-0.5:0.05:0.5);
[Y,Z] = meshgrid(-0.2:0.01:0.2,-0.2:0.01:0.2);
sizeMat = size(Y);
X = repmat(0.2,sizeMat(1),sizeMat(2));
oneSideOfCube_h = surf(X,Y,Z);
cubePoints = [X(:),Y(:),Z(:)];
cubePoints = [ cubePoints ...
    ; cubePoints * rotz(pi/2)...
    ; cubePoints * rotz(pi) ...
    ; cubePoints * rotz(3*pi/2) ...
    ; cubePoints * roty(pi/2) ...
    ; cubePoints * roty(-pi/2)];
% cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
cubePoints = cubePoints + repmat([2,0,1.5],size(cubePoints,1),1);
cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
%set(cube_h, 'Visible', 'off');





delta = 2*pi/steps;                                                         % Small angle change
epsilon = 0.1;                                                              % Threshold for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);                                              % Weighting matrix for the velocity vector

% 1.2) Allocate array data
m = zeros(total_steps,1);                                                   % Array for Measure of Manipulability
qMatrix = zeros(total_steps,6);                                             % Array for joint angles
qdot = zeros(total_steps,6);                                                % Array for joint velocities
theta = zeros(3,total_steps);                                               % Array for roll-pitch-yaw angles
x = zeros(3,total_steps);                                                   % Array for x-y-z trajectory
positionError = zeros(3,total_steps);                                       % For plotting trajectory error
angleError = zeros(3,total_steps);                                          % For plotting angle error

% 1.3) Set up forward trajectory, initial pose
s = lspb(0,1,steps);                                                        % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = 2;                                                             % Points in x (fixed)
    x(2,i) = (1-s(i))*-1.55 + s(i)*1.55;                                    % Points in y (move forward)
    x(3,i) = 1.5;                                                           % Points in z (fixed)

    theta(1,i) = 0;                                                         % Roll angle
    theta(2,i) = 5*pi/9;                                                    % Pitch angle
    theta(3,i) = 0;                                                         % Yaw angle
end

% 1.3) Set up backward trajectory
for i=1:steps
    x(1,steps+i) = 2;                                                       % Points in x (fixed)
    x(2,steps+i) = (1-s(i))*1.55 + s(i)*-1.55;                              % Points in y (move backward)
    x(3,steps+i) = 1.5;                                                     % Points in z (fixed)

    theta(1,steps+i) = 0;                                                   % Roll angle
    theta(2,steps+i) = 5*pi/9;                                              % Pitch angle
    theta(3,steps+i) = 0;                                                   % Yaw angle
end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1); zeros(1,3) 1];         % Transformation of first point
q0 = zeros(1,6);                                                            % Initial guess for joint angles
qMatrix(1,:) = titan.model.ikcon(T,q0);

% 1.4) Track the trajectory with RMRC
for i = 1:total_steps-1
    T = titan.model.fkine(qMatrix(i,:)).T;                                  % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                                           % Position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Desired rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                            % Rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric matrix
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2); S(1,3); S(2,1)];                            % Angular velocity
    deltaTheta = tr2rpy(Rd*Ra');                                            % RPY angle error
    xdot = W*[linear_velocity; angular_velocity];                           % End-effector velocity
    J = titan.model.jacob0(qMatrix(i,:));                                   % Jacobian at current state
    m(i) = sqrt(det(J*J'));                                                 % Manipulability measure
    if m(i) < epsilon                                                       % DLS damping if manipulability is low
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS inverse Jacobian
    qdot(i,:) = (invJ*xdot)';                                               % Joint velocities

    % Joint limit checks
    for j = 1:6
        if qMatrix(i,j) + deltaT*qdot(i,j) < titan.model.qlim(j,1)
            qdot(i,j) = 0;                                                  % Stop motor if below joint limit
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > titan.model.qlim(j,2)
            qdot(i,j) = 0;                                                  % Stop motor if above joint limit
        end
    end

    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                       % Update joint states
    positionError(:,i) = x(:,i+1) - T(1:3,4);                               % Position error tracking
    angleError(:,i) = deltaTheta;                                           % Angle error tracking

    % ---- Collision Detection ----
    % Get the current transformation matrices for each link
    q = qMatrix(i+1,:);
    tr = zeros(4,4,titan.model.n+1);
    tr(:,:,1) = titan.model.base;
    L = titan.model.links;
    for k = 1:titan.model.n
        tr(:,:,k+1) = tr(:,:,k) * trotz(q(k)) * transl(0,0,L(k).d) * transl(L(k).a,0,0) * trotx(L(k).alpha);
    end


end

% Plot the robot movement
tic

for i = 1:total_steps
    titan.model.animate(qMatrix(i,:));
    plot3(x(1,i), x(2,i), x(3,i), 'k.');
    drawnow

    % Check each link for collision

    q = titan.model.getpos();

    tr = zeros(4,4,titan.model.n+1);
    tr(:,:,1) = titan.model.base;
    L = titan.model.links;
    for i = 1 : titan.model.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    end

    for k = 2:6
        cubePointsAndOnes = [inv(tr(:,:,k)) * [cubePoints, ones(size(cubePoints,1),1)]']';
        updatedCubePoints = cubePointsAndOnes(:,1:3);
        algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoints(k,:), radii(k, :));
        pointsInside = find(algebraicDist < 1);
        if ~isempty(pointsInside)
            disp(['Collision detected: ', num2str(size(pointsInside,1)), ' points inside link ', num2str(k), ' ellipsoid']);
            stopped = true;
        end
    end
    % ----------------------------
    if stopped
        break
    end

end

disp(['Plot took ', num2str(toc), ' seconds'])


function defineObstacle(obsX, obsY, obsZ, a)
%% make the cube
% [Y,Z] = meshgrid(-0.5:0.05:0.5,-0.5:0.05:0.5);
[Y,Z] = meshgrid(-a:0.01:a,-a:0.01:a);
sizeMat = size(Y);
X = repmat(a,sizeMat(1),sizeMat(2));
oneSideOfCube_h = surf(X,Y,Z);
cubePoints = [X(:),Y(:),Z(:)];
cubePoints = [ cubePoints ...
    ; cubePoints * rotz(pi/2)...
    ; cubePoints * rotz(pi) ...
    ; cubePoints * rotz(3*pi/2) ...
    ; cubePoints * roty(pi/2) ...
    ; cubePoints * roty(-pi/2)];
% cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
cubePoints = cubePoints + repmat([obsX,obsY,obsZ],size(cubePoints,1),1);
cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
%set(cube_h, 'Visible', 'off');
end

%% dist2pts
%
% *Description:*  Function for find the distance between 2 or the same number of 3D points

%% Function Call
%
% *Inputs:*
%
% _pt1_ (many*(2||3||6) double) x,y || x,y,z cartesian point ||Q Joint angle
%
% _pt2_ (many*(2||3||6) double) x,y || x,y,z cartesian point ||Q Joint angle
%
% *Returns:*
%
% _dist_ (double) distance from pt1 to pt2

function dist=dist2pts(pt1,pt2)

%% Calculate distance (dist) between consecutive points
% If 2D
if size(pt1,2) == 2
    dist=sqrt((pt1(:,1)-pt2(:,1)).^2+...
        (pt1(:,2)-pt2(:,2)).^2);
    % If 3D
elseif size(pt1,2) == 3
    dist=sqrt((pt1(:,1)-pt2(:,1)).^2+...
        (pt1(:,2)-pt2(:,2)).^2+...
        (pt1(:,3)-pt2(:,3)).^2);
    % If 6D like two poses
elseif size(pt1,2) == 6
    dist=sqrt((pt1(:,1)-pt2(:,1)).^2+...
        (pt1(:,2)-pt2(:,2)).^2+...
        (pt1(:,3)-pt2(:,3)).^2+...
        (pt1(:,4)-pt2(:,4)).^2+...
        (pt1(:,5)-pt2(:,5)).^2+...
        (pt1(:,6)-pt2(:,6)).^2);
end
end

%% GetAlgebraicDist
% determine the algebraic distance given a set of points and the center
% point and radii of an elipsoid
% *Inputs:*
%
% _points_ (many*(2||3||6) double) x,y,z cartesian point
%
% _centerPoint_ (1 * 3 double) xc,yc,zc of an ellipsoid
%
% _radii_ (1 * 3 double) a,b,c of an ellipsoid
%
% *Returns:*
%
% _algebraicDist_ (many*1 double) algebraic distance for the ellipsoid

function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
    + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
    + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end