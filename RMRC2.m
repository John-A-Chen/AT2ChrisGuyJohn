% 1.1) Set parameters for the simulation
ur3e = UR3e();                                                          % Load robot model
t = 1;                                                                      % Total time for one direction (s)
deltaT = 0.02;                                                              % Control frequency
steps = t/deltaT;                                                           % Number of steps for one direction
total_steps = 2*steps;                                                      % Total steps for forward and backward
S = 5;

surf([-S, -S; S, S], ...
    [-S, S; -S, S], [0, 0; 0, 0], ...
    'CData', imread('concrete.jpg'), 'FaceColor', 'texturemap');
surf([S, S; S, S], ...
    [-S, S; -S, S], [S, S; 0, 0], ...
    'CData', imread('IMG_7414.jpg'), 'FaceColor', 'texturemap');
surf([-S, S; -S, S], ...
    [S, S; S, S], [S, S; 0, 0], ...
    'CData', imread('IMG_7413.jpg'), 'FaceColor', 'texturemap');
PlaceObject('environment.PLY',[0,0,0]);
axis equal;

delta = 2*pi/steps;                                                         % Small angle change
epsilon = 0.1;                                                              % Threshold for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);                                              % Weighting matrix for the velocity vector

% 1.2) Allocate array data
m = zeros(total_steps,1);                                                   % Array for Measure of Manipulability
qMatrix = zeros(total_steps,7);                                             % Array for joint angles
qdot = zeros(total_steps,7);                                                % Array for joint velocities
theta = zeros(3,total_steps);                                               % Array for roll-pitch-yaw angles
x = zeros(3,total_steps);                                                   % Array for x-y-z trajectory
positionError = zeros(3,total_steps);                                       % For plotting trajectory error
angleError = zeros(3,total_steps);                                          % For plotting angle error

% 1.3) Set up forward trajectory, initial pose
s = lspb(0,1,steps);                                                        % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = 0.75;                                                          % Points in x (fixed)
    x(2,i) = (1-s(i))*-1 + s(i)*1;                                          % Points in y (move forward)
    x(3,i) = 0.75;                                                          % Points in z (fixed)

    theta(1,i) = 0;                                                         % Roll angle
    theta(2,i) = 5*pi/9;                                                    % Pitch angle
    theta(3,i) = 0;                                                         % Yaw angle
end

% 1.3) Set up backward trajectory
for i=1:steps
    x(1,steps+i) = 0.75;                                                    % Points in x (fixed)
    x(2,steps+i) = (1-s(i))*1 + s(i)*-1;                                    % Points in y (move backward)
    x(3,steps+i) = 0.75;                                                    % Points in z (fixed)

    theta(1,steps+i) = 0;                                                   % Roll angle
    theta(2,steps+i) = 5*pi/9;                                              % Pitch angle
    theta(3,steps+i) = 0;                                                   % Yaw angle
end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1); zeros(1,3) 1];         % Transformation of first point
q0 = zeros(1,7);                                                            % Initial guess for joint angles
qMatrix(1,:) = ur3e.model.ikcon(T,q0);                                      % Solve joint angles to achieve first waypoint

% 1.4) Track the trajectory with RMRC
for i = 1:total_steps-1
    T = ur3e.model.fkine(qMatrix(i,:)).T;                                  % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                                           % Position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Desired rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                            % Rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric matrix
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2); S(1,3); S(2,1)];                            % Angular velocity
    deltaTheta = tr2rpy(Rd*Ra');                                            % RPY angle error
    xdot = W*[linear_velocity; angular_velocity];                           % End-effector velocity
    J = ur3e.model.jacob0(qMatrix(i,:));                                   % Jacobian at current state
    m(i) = sqrt(det(J*J'));                                                 % Manipulability measure
    if m(i) < epsilon                                                       % DLS damping if manipulability is low
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(7))*J';                                   % DLS inverse Jacobian
    qdot(i,:) = (invJ*xdot)';                                               % Joint velocities

    % Joint limit checks
    for j = 1:7
        if qMatrix(i,j) + deltaT*qdot(i,j) < ur3e.model.qlim(j,1)
            qdot(i,j) = 0;                                                  % Stop motor if below joint limit
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > ur3e.model.qlim(j,2)
            qdot(i,j) = 0;                                                  % Stop motor if above joint limit
        end
    end

    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                       % Update joint states
    positionError(:,i) = x(:,i+1) - T(1:3,4);                               % Position error tracking
    angleError(:,i) = deltaTheta;                                           % Angle error tracking
end

% Plot the robot movement
tic
hold on
% figure(1)
plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
% ur3e.model.plot(qMatrix,'trail','r-')

for i = 1:total_steps
    ur3e.model.animate(qMatrix(i,:));
    % plot3(x(1,i), x(2,i), x(3,i), 'k.');
    drawnow
end

disp(['Plot took ', num2str(toc), ' seconds'])
