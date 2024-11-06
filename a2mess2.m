clf;
axis equal;
hold on;
view(3);
titan = KukaTitan(transl(-0.75, 0.5, 0.05));
ur3e = UR3e(transl(2.5, 1.75, 0.925));
surf([-5, -5; 5, 5], ...
    [-5, 5; -5, 5], [0, 0; 0, 0], ...
    'CData', imread('concrete.jpg'), 'FaceColor', 'texturemap');
surf([5, 5; 5, 5], ...
    [-5, 5; -5, 5], [5, 5; 0, 0], ...
    'CData', imread('IMG_7414.jpg'), 'FaceColor', 'texturemap');
surf([-5, 5; -5, 5], ...
    [5, 5; 5, 5], [5, 5; 0, 0], ...
    'CData', imread('IMG_7413.jpg'), 'FaceColor', 'texturemap');
light("Style","local","Position",[-10 -10 3]);
PlaceObject('environment.PLY', [0, 0, 0]);

startPositions = ...
    [-1.0, 3.5, 1.3;
    -1.4, 3.5, 1.3;
    -1.8, 3.5, 1.3];

weldPosition = ...
    [2, 2, 1.75;
    2, 2, 1.75;
    2, 2, 1.75];

endPositions = ...
    [-0.375, 2, 0.9;
    0.0, 2, 0.9;
    0.375, 2, 0.9];

t1 = 1;                                                         % Total time for one direction (s)
deltaT1 = 0.02;                                                 % Control frequency
steps1 = t1 / deltaT1;                                          % Number of steps for one direction
total_steps1 = 2 * steps1;                                      % Total steps for forward and backward
S1 = 5;
delta1 = 2 * pi / steps1;                                       % Small angle change
epsilon1 = 0.1;                                                 % Threshold for manipulability/Damped Least Squares
W1 = diag([1 1 1 0.1 0.1 0.1]);                                 % Weighting matrix for the velocity vector

hflange = cell(1, size(startPositions, 1));
for i = 1:size(startPositions, 1)
    hflange{i} = PlaceObject('flange.ply', startPositions(i, :));
end

% Allocate array data
qMatrix = zeros(total_steps1, 7);                               % Array for UR3e joint angles
qMatrix2 = zeros(total_steps1, 6);                              % Array for Kuka Titan joint angles
qdot = zeros(total_steps1, 7);                                  % Array for UR3e joint velocities
qdot2 = zeros(total_steps1, 6);                                 % Array for Kuka Titan joint velocities
theta = zeros(3, total_steps1);                                 % Array for UR3e roll-pitch-yaw angles
theta2 = zeros(3, total_steps1);                                % Array for Kuka Titan roll-pitch-yaw angles
x = zeros(3, total_steps1);                                     % Array for UR3e x-y-z trajectory
x2 = zeros(3, total_steps1);                                    % Array for Kuka Titan x-y-z trajectory

% UR3e movement with RMRC (Circular Trajectory)
radius = 0.15;                                                  % Radius of the circle
center = [2; 2; 1.75];                                          % Center of the circle in (x, y, z) coordinates
theta(1, :) = 0;                                               % Roll angle (fixed)
theta(2, :) = 5 * pi / 9;                                      % Pitch angle (fixed)
theta(3, :) = pi;                                              % Yaw angle (fixed)

s1 = lspb(0, 4 * pi, steps1);                                  % Define angular positions over time for one full circle
for i = 1:steps1
    x(1, i) = center(1) + radius * cos(s1(i));                % x-coordinate for circular path
    x(2, i) = center(2);                                       % y-coordinate (fixed at center)
    x(3, i) = center(3) + radius * sin(s1(i));                % z-coordinate for circular path
end

% Concatenate to complete the circular trajectory by mirroring in reverse
x = [x, x(:, end:-1:1)];                                       % Extend trajectory to move forward and then reverse

T = [rpy2r(theta(1, 1), theta(2, 1), theta(3, 1)), ...
    x(:, 1); zeros(1, 3) 1];                                  % Initial transformation
q0 = zeros(1, 7);                                              % Initial guess for joint angles
qMatrix(1, :) = ur3e.model.ikcon(T, q0);                      % Solve joint angles for first waypoint

% RMRC loop for UR3e following the circular trajectory
for i = 1:steps1 - 1
    T = ur3e.model.fkine(qMatrix(i, :)).T;                    % Get forward transformation at current joint state
    deltaX = x(:, i + 1) - T(1:3, 4);                          % Position error from next waypoint
    Rd = rpy2r(theta(1, i + 1), theta(2, i + 1), theta(3, i + 1)); % Desired rotation matrix
    Ra = T(1:3, 1:3);                                          % Current end-effector rotation matrix
    Rdot = (1 / deltaT1) * (Rd - Ra);                          % Rotation matrix error
    S1 = Rdot * Ra';                                           % Skew symmetric matrix
    linear_velocity = (1 / deltaT1) * deltaX;
    angular_velocity = [S1(3, 2); S1(1, 3); S1(2, 1)];       % Angular velocity
    xdot = W1 * [linear_velocity; angular_velocity];           % End-effector velocity
    J = ur3e.model.jacob0(qMatrix(i, :));                     % Jacobian at current state

    % Damped Least Squares (DLS) for low manipulability
    m = sqrt(det(J * J'));
    if m < epsilon1
        lambda = (1 - m / epsilon1) * 5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J' * J + lambda * eye(7)) * J';                % DLS inverse Jacobian
    qdot(i, :) = (invJ * xdot)';                               % Joint velocities
    qMatrix(i + 1, :) = qMatrix(i, :) + deltaT1 * qdot(i, :);
end

% Kuka Titan LSPB movement between brick positions
s2 = lspb(0, 1, steps1);                                       % Trapezoidal trajectory scalar

% Check array size and ensure the index doesn't exceed the number of startPositions
numPositions = min([steps1, size(startPositions, 1)]);

for i = 1:numPositions
    % First segment: Move from start to weld position
    x2(:, i) = (1 - s2(i)) * startPositions(i, :) + s2(i) * weldPosition(i, :);
    theta2(:, i) = [0; 5 * pi / 9; 0];
end
for i = 1:numPositions
    % Second segment: Move from weld position to end position
    x2(:, steps1 + i) = (1 - s2(i)) * weldPosition(i, :) + s2(i) * endPositions(i, :);
    theta2(:, steps1 + i) = [0; 5 * pi / 9; 0];
end

% Set up initial transformation and joint angles for Kuka
T2 = [rpy2r(theta2(1, 1), theta2(2, 1), theta2(3, 1)), ...
    x2(:, 1); zeros(1, 3) 1];                                % Transformation of first point
q1 = zeros(1, 6);                                             % Initial guess for joint angles
qMatrix2(1, :) = titan.model.ikcon(T2, q1);                  % Solve joint angles to achieve first waypoint

% RMRC loop for Kuka Titan
for i = 1:total_steps1 - 1
    T2 = titan.model.fkine(qMatrix2(i, :)).T;                % Get forward transformation at current joint state
    deltaX2 = x2(:, i + 1) - T2(1:3, 4);                     % Position error from next waypoint
    Rd2 = rpy2r(theta2(1, i + 1), theta2(2, i + 1), theta2(3, i + 1)); % Desired rotation matrix
    Ra2 = T2(1:3, 1:3);                                       % Current end-effector rotation matrix
    Rdot2 = (1 / deltaT1) * (Rd2 - Ra2);                      % Rotation matrix error
    S2 = Rdot2 * Ra2';                                        % Skew symmetric matrix
    linear_velocity2 = (1 / deltaT1) * deltaX2;
    angular_velocity2 = [S2(3, 2); S2(1, 3); S2(2, 1)];      % Angular velocity
    xdot2 = W1 * [linear_velocity2; angular_velocity2];       % End-effector velocity
    J2 = titan.model.jacob0(qMatrix2(i, :));                  % Jacobian at current state

    % Damped Least Squares for Kuka Titan
    m2 = sqrt(det(J2 * J2'));
    if m2 < epsilon1
        lambda2 = (1 - m2 / epsilon1) * 5E-2;
    else
        lambda2 = 0;
    end
    invJ2 = inv(J2' * J2 + lambda2 * eye(6)) * J2';           % DLS inverse Jacobian
    qdot2(i, :) = (invJ2 * xdot2)';                            % Joint velocities
    qMatrix2(i + 1, :) = qMatrix2(i, :) + deltaT1 * qdot2(i, :); % Update joint angles
end

% Visualize the simulation for both robots
hold on;
for i = 1:total_steps1
    ur3e.model.animate(qMatrix(i, :));                                % Plot UR3e motion
    titan.model.animate(qMatrix2(i, :));                             % Plot Kuka Titan motion
    drawnow();                                         % Wait before the next step
end

