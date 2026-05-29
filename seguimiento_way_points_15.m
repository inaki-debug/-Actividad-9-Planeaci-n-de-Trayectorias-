%% EXAMPLE: Differential drive vehicle following waypoints using the 
% Pure Pursuit algorithm
%
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:200;         % Time array

initPose = [4;1;2.35619];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Define waypoints
waypoints = [3.8,1.7; 3.9,2; 4,2.4; 4.3,2.6; 4.8,3.1; 4.8,3.3; 5,3.7; 5.1,3.8; 5.2,4.1; 5.3,4.2; 5.8,4.3; 5.9,4.5; 5.7,4.8; 5.6,5; 5.8,5.1; 5.9,5.4; 6.1,5.5; 6.3,5.8; 6.4,6.1; 6.5,6.4; 6.5,6.8; 6.5,7; 6.5,7.3; 6.5,7.5; 6.5,7.7; 6.6,8; 6.7,8.4; 6.7,8.6; 6.6,8.8; 6.8,9; 7,9.5; 7.1,9.8; 7.1,10; 7.1,10.3; 7,10.5; 6.7,10.6; 6.4,10.6; 6.1,10.5; 6,10.3; 5.7,10; 5.5,10.3; 5.4,10.5; 5.3,10.6; 4.9,10.7; 4.7,10.7; 4.5,10.7; 4.3,10.6; 4.1,10.5; 4.1,10.8; 4,11; 3.9,11.1; 3.6,11.1; 3.4,11.1; 3.1,11; 3.3,10.6; 3.1,10.9; 2.9,10.9; 2.7,10.8; 2.3,10.7; 2,10.5; 1.8,10.3; 1.6,10.1; 1.5,10.3; 1.3,10.4; 1.1,10.5; 0.9,10.5; 0.7,10.5; 0.5,10.3; 0.5,10.1; 0.5,10; 0.5,9.8; 0.5,9.6; 0.6,9.2; 0.8,9; 0.9,8.9; 0.8,8.6; 1,8; 1,7.6; 1,7.2; 1,6.8; 0.9,6.5; 1.2,6; 1.4,5.5; 1.7,5; 1.6,4.4; 1.6,4.4; 2,4.5; 2,4.5; 2.3,4.4; 2.4,4.2; 2.7,3.8; 2.8,3.5; 2.9,3.2; 3.1,2.9; 3.5,2.4; 3.6,2.2; 3.7,1.8; 3.2,3.2; 3.4,3.5; 3.6,3.6; 3.9,3.5; 4.6,3.4; 4.1,3.5; 3.8,3.7; 3.9,4.1; 4.2,4.5; 4.3,4.7; 4.5,4.9; 4,4.3; 3.8,4.1; 3.6,4.3; 3.1,4.9; 2.9,5.2; 4.7,6.8; 4.9,7; 5.4,7.1; 5.6,7.2; 5.7,7.5; 5.2,7.8; 5.3,8.2; 5,8.7; 4.8,8.7; 4.2,8; 4.1,7.8; 3.9,7.6; 3.6,7.6; 3.3,8; 3.2,8.2; 3.1,8.4; 2.8,8.7; 2.1,8.4; 2.2,8; 2.3,7.8; 1.9,7.7; 1.9,7.2; 2.3,7; 2.7,6.7];% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.30;
controller.DesiredLinearVelocity = 0.40;
controller.MaxAngularVelocity = 100;

%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef)
 
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints)
    waitfor(r);
    
end
