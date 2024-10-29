% Set parameters for the simulation
ur3e = UR3e();                                                 % Load robot model 1

t = 2;                                                                      % Total time for one direction (s)
deltaT = 0.02;                                                              % Control frequency
steps = t/deltaT;                                                           % Number of steps for one direction                                                      % Total steps for forward and backward
epsilon = 0.1;                                                              % Threshold for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);                                              % Weighting matrix for the velocity vector

% Allocate array data
qMatrix = zeros(steps, 7);                                             % Array for UR3e joint angles
qdot = zeros(steps, 7);                                                % Array for UR3e joint velocities
theta = zeros(3, steps);                                               % Array for UR3e roll-pitch-yaw angles
x = zeros(3, steps);                                                   % Array for UR3e x-y-z trajectory

view(180, 45)

%% UR3e movement with RMRC (Circular Trajectory)
radius = 0.15;                  % Radius of the circle
center = [0; 1; 1];        % Center of the circle in (x, y, z) coordinates, 0.5m away in Z
theta(1,:) = 0;              % Roll angle (fixed)
theta(2,:) = 5*pi/9;           % Pitch angle (fixed)
theta(3,:) = 5*pi/9;              % Yaw angle (fixed)

s1 = lspb(0, 2*pi, steps); % Define angular positions over time for one full circle
for i = 1:steps
    x(1,i) = center(1) + radius * cos(s1(i)); % x-coordinate for circular path
    x(2,i) = center(2);                        % y-coordinate (fixed at center)
    x(3,i) = center(3) + radius * sin(s1(i)); % z-coordinate for circular path
end

% Concatenate to complete the circular trajectory by mirroring in reverse
x = [x, x(:, end:-1:1)];  % Extend trajectory to move forward and then reverse

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1); zeros(1,3) 1];  % Initial transformation
q0 = zeros(1, 7); % Initial guess for joint angles
qMatrix(1, :) = ur3e.model.ikcon(T, q0); % Solve joint angles for first waypoint

% Check size of x to confirm number of trajectory points
disp(size(x)); % Display size of x for debugging

% RMRC loop for UR3e following the circular trajectory
for i = 1:steps - 1
    T = ur3e.model.fkine(qMatrix(i,:)).T;                                   % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                                           % Position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Desired rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                            % Rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric matrix
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2); S(1,3); S(2,1)];                            % Angular velocity
    xdot = W*[linear_velocity; angular_velocity];                           % End-effector velocity
    J = ur3e.model.jacob0(qMatrix(i,:));                                    % Jacobian at current state

    % Damped Least Squares (DLS) for low manipulability
    m = sqrt(det(J*J'));
    if m < epsilon
        lambda = (1 - m/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(7))*J';                                   % DLS inverse Jacobian
    qdot(i,:) = (invJ*xdot)';                                               % Joint velocities
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:); 
end

%% Plot the movement
hold on;
for i = 1:steps-1
    ur3e.model.animate(qMatrix(i, :));                                       % Animate UR3e robot
    plot3(x(1, i), x(2, i), x(3, i), 'k.');
    drawnow();
end
