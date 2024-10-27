% 1.1) Set parameters for the simulation
titan = KukaTitan;        % Load robot model
t = 1;              % Total time for one direction (s)
deltaT = 0.02;      % Control frequency
steps = t/deltaT;   % Number of steps for one direction

hold on
c = [1.8;0;1.5];
r = [0.8,0.5,0.5];
ellipsoid(c(1),c(2),c(3),r(1),r(2),r(3));


delta = 2*pi/steps; % Small angle change
epsilon = 0.1;      % Threshold for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector

% 1.2) Allocate array data
m = zeros(steps,1);        % Array for Measure of Manipulability
qMatrix = zeros(steps,6);  % Array for joint angles
qdot = zeros(steps,6);     % Array for joint velocities
theta = zeros(3,steps);    % Array for roll-pitch-yaw angles
x = zeros(3,steps);        % Array for x-y-z trajectory
positionError = zeros(3,steps); % For plotting trajectory error
angleError = zeros(3,steps);    % For plotting angle error

% 1.3) Set up forward trajectory, initial pose
% s = lspb(0,1,steps);    % Trapezoidal trajectory scalar
s = linspace(0, 1, steps);
x1 = [2; -1.55; 1.5];
x2 = [2; 1.55; 1.5];
theta1 = [0;5*pi/9;0];
theta2 = [0;5*pi/9;0];
for i=1:steps
    x(:,i) = (1-s(i))*x1 + s(i)*x2;
    % if norm(x(:,i)-c) < (r(1)+0.1)
    d = (x(:,i)-c)' * diag(r.^-2) * (x(:,i)-c);
    if d < 1
        if norm(cross(x2 - x1, x(:,i)-c)) == 0
            x(3,i) = x(3,i) + 0.1;
        end
        x(:,i) = c + (d^-0.5) * (x(:,i)-c);
    end
    theta(:,i) = (1-s(i))*theta1 + s(i)*theta2;
end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1); zeros(1,3) 1];   % Transformation of first point
q0 = zeros(1,6);                                                      % Initial guess for joint angles
qMatrix(1,:) = titan.model.ikcon(T,q0);                                       % Solve joint angles to achieve first waypoint

% 1.4) Track the trajectory with RMRC
for i = 1:steps-1
    T = titan.model.fkine(qMatrix(i,:)).T;                                  % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);
    % deltaX = x(:,i+1) - T(1:3,4) + 2*(x(:,i+1) - c);                                    % Position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));              % Desired rotation matrix
    Ra = T(1:3,1:3);                                                 % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                     % Rotation matrix error
    S = Rdot*Ra';                                                    % Skew symmetric matrix
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2); S(1,3); S(2,1)];                     % Angular velocity
    deltaTheta = tr2rpy(Rd*Ra');                                      % RPY angle error
    xdot = W*[linear_velocity; angular_velocity];                    % End-effector velocity
    J = titan.model.jacob0(qMatrix(i,:));                                   % Jacobian at current state
    m(i) = sqrt(det(J*J'));                                          % Manipulability measure
    if m(i) < epsilon                                                % DLS damping if manipulability is low
        lambda = (1 - m(i)/epsilon)*5e-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                            % DLS inverse Jacobian
    qdot(i,:) = (invJ*xdot)';                                        % Joint velocities

    % Joint limit checks
    for j = 1:6
        if qMatrix(i,j) + deltaT*qdot(i,j) < titan.model.qlim(j,1)
            qdot(i,j) = 0;                                           % Stop motor if below joint limit
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > titan.model.qlim(j,2)
            qdot(i,j) = 0;                                           % Stop motor if above joint limit
        end
    end
    
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                % Update joint states
    positionError(:,i) = x(:,i+1) - T(1:3,4);                        % Position error tracking
    angleError(:,i) = deltaTheta;                                    % Angle error tracking
end

% Plot the robot movement
tic
% hold on
% figure(1)
% plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
% titan.model.plot(qMatrix,'trail','r-')

for i = 1:steps
    titan.model.animate(qMatrix(i,:));
    % plot3(x(1,i), x(2,i), x(3,i), 'k.');
    x(:,i) = titan.model.fkine(qMatrix(i,:)).t;
    plot3(x(1,i), x(2,i), x(3,i), 'k.');
    drawnow
end

disp(['Plot took ', num2str(toc), ' seconds'])
% disp(xdot)