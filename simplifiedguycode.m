%% make the cube
[Y,Z] = meshgrid(-0.5:0.05:0.5,-0.5:0.05:0.5);
sizeMat = size(Y);
X = repmat(0.5,sizeMat(1),sizeMat(2));
oneSideOfCube_h = surf(X,Y,Z);
cubePoints = [X(:),Y(:),Z(:)];
cubePoints = [ cubePoints ...
    ; cubePoints * rotz(pi/2)...
    ; cubePoints * rotz(pi) ...
    ; cubePoints * rotz(3*pi/2) ...
    ; cubePoints * roty(pi/2) ...
    ; cubePoints * roty(-pi/2)];
cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
cubePoints = cubePoints + repmat([0.75,0,-0.25],size(cubePoints,1),1);
cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
set(cube_h, 'Visible', 'off');
axis equal;

%% robor
robot = UR3e;  % Define the robot model
centerPoints = [
    0, 0, 0;       % Link 1
    0, 0, 0.14;    % Link 2
    0, 0, 0.14;    % Link 3
    0, 0, 0;       % Link 4
    0, 0, 0;       % Link 5
    0, 0, 0;       % Link 6
    ];

radii = [
    0.2, 0.2, 0.2;   % Link 1
    0.28, 0.2, 0.28; % Link 2
    0.4, 0.2, 0.28;  % Link 3
    0.2, 0.2, 0.2;   % Link 4
    0.2, 0.2, 0.2;   % Link 5
    0.2, 0.2, 0.2;   % Link 6
    ];
q = robot.model.getpos();

%% put elipsoid on robor
function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
    + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
    + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end
tr = zeros(4,4,robot.model.n+1);
tr(:,:,1) = robot.model.base;
L = robot.model.links;
for i = 1 : robot.model.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end
for i = 2: 6
    warning off;
    cubePointsAndOnes = (inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]')';
    updatedCubePoints = cubePointsAndOnes(:,1:3);
    algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoints(i,:), radii(i, :));
    pointsInside = find(algebraicDist < 1);
    disp(['There are ', num2str(size(pointsInside,1)),' points inside joint ',num2str(i),' ellipsoid']);
    warning on;
end
