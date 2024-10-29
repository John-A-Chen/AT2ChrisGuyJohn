function rmrc5()
% Set parameters for the simulation
ur3e = UR3e(transl(2,0,0));                                                 % Load robot model 1
titan = KukaTitan();                                                        % Load robot model 2

t = 1;                                                                      % Total time for one direction (s)
deltaT = 0.02;                                                              % Control frequency
steps = t/deltaT;                                                           % Number of steps for one direction
total_steps = 2*steps;                                                      % Total steps for forward and backward
S = 5;
delta = 2*pi/steps;                                                         % Small angle change
epsilon = 0.1;                                                              % Threshold for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);                                              % Weighting matrix for the velocity vector

% Allocate array data
qMatrix = zeros(total_steps,7);                                             % Array for UR3e joint angles
qMatrix2 = zeros(total_steps,6);                                            % Array for Kuka Titan joint angles
qdot = zeros(total_steps,7);                                                % Array for UR3e joint velocities
qdot2 = zeros(total_steps,6);
theta = zeros(3,total_steps);                                               % Array for UR3e roll-pitch-yaw angles
theta2 = zeros(3,total_steps);                                              % Array for Kuka Titan roll-pitch-yaw angles
x = zeros(3,total_steps);                                                   % Array for UR3e x-y-z trajectory
x2 = zeros(3,total_steps);                                                  % Array for Kuka Titan x-y-z trajectory

% UR3e movement with RMRC
s1 = lspb(0,1,steps);                                                       % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = 1;                                                             % Points in x (fixed)
    x(2,i) = (1-s1(i))*-1 + s1(i)*1;                                        % Points in y (move forward)
    x(3,i) = 1;                                                             % Points in z (fixed)
    theta(1,i) = 0;                                                         % Roll angle
    theta(2,i) = 5*pi/9;                                                    % Pitch angle
    theta(3,i) = 0;                                                         % Yaw angle
end
for i=1:steps
    x(1,steps+i) = 1;                                                       % Points in x (fixed)
    x(2,steps+i) = (1-s1(i))*1 + s1(i)*-1;                                  % Points in y (move backward)
    x(3,steps+i) = 1;                                                       % Points in z (fixed)
    theta(1,steps+i) = 0;                                                   % Roll angle
    theta(2,steps+i) = 5*pi/9;                                              % Pitch angle
    theta(3,steps+i) = 0;                                                   % Yaw angle
end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1); zeros(1,3) 1];         % Transformation of first point
q0 = zeros(1,7);                                                            % Initial guess for joint angles
qMatrix(1,:) = ur3e.model.ikcon(T,q0);                                      % Solve joint angles to achieve first waypoint

% RMRC loop for UR3e
for i = 1:total_steps-1
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
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                       % Update joint states
end

% Kuka movement with RMRC
s2 = lspb(0,1,steps);                                                        % Trapezoidal trajectory scalar
for i=1:steps
    x2(1,i) = 2;                                                            % Points in x (fixed)
    x2(2,i) = (1-s2(i))*-1.55 + s2(i)*1.55;                                 % Points in y (move forward)
    x2(3,i) = 1.5;                                                          % Points in z (fixed)
    theta2(1,i) = 0;                                                        % Roll angle
    theta2(2,i) = 5*pi/9;                                                   % Pitch angle
    theta2(3,i) = 0;                                                        % Yaw angle
end
for i=1:steps
    x2(1,steps+i) = 2;                                                      % Points in x (fixed)
    x2(2,steps+i) = (1-s2(i))*1.55 + s2(i)*-1.55;                           % Points in y (move backward)
    x2(3,steps+i) = 1.5;                                                    % Points in z (fixed)
    theta2(1,steps+i) = 0;                                                  % Roll angle
    theta2(2,steps+i) = 5*pi/9;                                             % Pitch angle
    theta2(3,steps+i) = 0;                                                  % Yaw angle
end

T2 = [rpy2r(theta2(1,1),theta2(2,1),theta2(3,1)) x2(:,1); zeros(1,3) 1];    % Transformation of first point
q1 = zeros(1,6);                                                            % Initial guess for joint angles
qMatrix2(1,:) = titan.model.ikcon(T2,q1);                                   % Solve joint angles to achieve first waypoint

% RMRC loop for Kuka Titan
for i = 1:total_steps-1
    T2 = titan.model.fkine(qMatrix2(i,:)).T;                                % Get forward transformation at current joint state
    deltaX2 = x2(:,i+1) - T2(1:3,4);                                        % Position error from next waypoint
    Rd2 = rpy2r(theta2(1,i+1),theta2(2,i+1),theta2(3,i+1));                 % Desired rotation matrix
    Ra2 = T2(1:3,1:3);                                                      % Current end-effector rotation matrix
    Rdot2 = (1/deltaT)*(Rd2 - Ra2);                                         % Rotation matrix error
    S2 = Rdot2*Ra2';                                                        % Skew symmetric matrix
    linear_velocity2 = (1/deltaT)*deltaX2;
    angular_velocity2 = [S2(3,2); S2(1,3); S2(2,1)];                        % Angular velocity
    xdot2 = W*[linear_velocity2; angular_velocity2];                        % End-effector velocity
    J2 = titan.model.jacob0(qMatrix2(i,:));                                 % Jacobian at current state

    % Damped Least Squares (DLS) for low manipulability
    m2 = sqrt(det(J2*J2'));
    if m2 < epsilon
        lambda = (1 - m2/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ2 = inv(J2'*J2 + lambda *eye(6))*J2';                               % DLS inverse Jacobian
    qdot2(i,:) = (invJ2*xdot2)';                                            % Joint velocities
    qMatrix2(i+1,:) = qMatrix2(i,:) + deltaT*qdot2(i,:);                    % Update joint states
end

% Plot the UR3e movement
hold on;
for i = 1:total_steps
    ur3e.model.animate(qMatrix(i,:));                                       % Animate UR3e robot
    titan.model.animate(qMatrix2(i,:));                                     % Animate Kuka Titan robot
    drawnow();
end
end