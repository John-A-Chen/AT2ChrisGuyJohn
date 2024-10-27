% Create camera object (replace 'CameraClass' with your actual camera class)
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, 'resolution', [1280 1024]);

% Define the world points (P) and desired image points (uv_star)
P = mkgrid(2, 0.5);  % Generate a 2x2 grid of points on the XY plane
uv_star = [640, 512; 680, 512; 640, 552; 680, 552];  % Desired image points

% Initial pose of the camera
T0 = transl(0.5, 0.5, 2.0);  % Translate camera to an initial position

% Final desired camera pose
Tf = transl(0, 0, 1.0);  % Move camera towards the object

% Create an instance of VisualServo (subclass IBVS would be required)
vs = VisualServo(cam, 'niter', 100, 'P', P, 'pstar', uv_star, 'T0', T0, 'Tf', Tf, 'fps', 10);

% Run the visual servo simulation for 50 steps
vs.run(50);

% Plot the results
figure;
vs.plot_p();      % Plot feature trajectories on the image plane

figure;
vs.plot_vel();    % Plot camera velocity over time

figure;
vs.plot_camera(); % Plot camera position and orientation over time

figure;
vs.plot_error();  % Plot feature error over time

% Optionally, plot all results in separate figures
vs.plot_all();
