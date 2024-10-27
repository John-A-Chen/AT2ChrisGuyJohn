q = robot.model.getpos()

tr = zeros(4,4,robot.model.n+1);
tr(:,:,1) = robot.model.base;
L = robot.model.links;
for i = 1 : robot.model.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

for i = 2: 6
    cubePointsAndOnes = [inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]']';
    updatedCubePoints = cubePointsAndOnes(:,1:3);
    algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoints(i,:), radii(i, :));
    pointsInside = find(algebraicDist < 1);
    disp(['There are ', num2str(size(pointsInside,1)),' points inside joint ',num2str(i),' ellipsoid']);
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