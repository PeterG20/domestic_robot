%% Import the Robot and add the Virtual End Effector
% Clears all variables and command window
clear
clc

% Imports two robot models from URDF files
robot = importrobot('Tinkerkit_model/tinkerkit4DOF.urdf');
robot2 = importrobot('Tinkerkit_model/tinkerkit.urdf');

% Gets the number of joints in the robot model
numJoints = numel(homeConfiguration(robot));

% Adds a virtual end effector to the robot model at a specified offset
eeOffset=0.16;
eeBody=robotics.RigidBody('end_effector');
setFixedTransform(eeBody.Joint, trvec2tform([0 0 eeOffset]));
addBody(robot,eeBody,'link4');

% Clears command window
clc

%% Set HOME position
% Gets home configurations for both robot models
config = homeConfiguration(robot);
config2 = homeConfiguration(robot2);

% Sets initial joint positions for the first robot model
config(1).JointPosition=pi/2;
config(2).JointPosition=pi/2;
config(3).JointPosition=pi/4;
config(4).JointPosition=pi/4;

% Displays the robot model in the home configuration
show(robot, config);
title('Braccio: HOME Configuration')
axis([-0.3 0.3 -0.3 0.3 -0.1 0.5]);
hold on

%% Detect Points to be Reached and Trajectory Tracking
% Defines waypoints for the trajectory
adHome    = [0.2 0 0.2];
adStartPoint    = [0.32 0 0.1];
adBallPoint     = [0.32 0 -0.05];
adPoint         = [0.25 0 0.1];
adGoalPoint     = [0.20 0 0.2];
adEndPoint      = [0.20 0 -0.05];
% x y z
adWaypoints     = [ adHome;
                    adStartPoint;
                    adBallPoint;
                    adPoint;
                    adGoalPoint;
                    adEndPoint];

%Positioning
adHome    = [0.2 0 0.2];

% x y z
adWaypoints     = [ adHome];

wayPoints=adWaypoints;

% Plots the waypoints
plot3(wayPoints(:,1),wayPoints(:,2),wayPoints(:,3),'.','MarkerSize',40,  'MarkerEdgeColor','k'); % waypoints of the End Effector
hold on

% Computes a smooth spline trajectory through the waypoints
traj=cscvn(wayPoints');
fnplt(traj,'r',2);
grid on
hold off

%% Inverse Kinematics Trajectory
% Initializes inverse kinematics solver for the robot model
ik = robotics.InverseKinematics('RigidBodyTree',robot);
ik.SolverAlgorithm = 'LevenbergMarquardt';
weights = [0 0 0 1 1 1];

% Computes inverse kinematics solution for each point on the trajectory
initialguess = config;
[n,~]=size(wayPoints);
totalPoints=n*20;

x=linspace(0,traj.breaks(end),totalPoints);
eePos=ppval(traj,x);

for idx = 1:size(eePos,2)
    tform = trvec2tform(eePos(:,idx)');
    configSoln(idx,:) = ik('end_effector',tform,weights,initialguess);
    initialguess = configSoln(idx,:);
end

%% Trajectory tracking
% Shows the trajectory tracking of the robot model in a new figure
figure
title('Waypoints Tracking')
config2(5).JointPosition=0;
config2(6).JointPosition=73*pi/180;
for idx = 1:size(eePos,2)
    config2(1).JointPosition=configSoln(idx,1).JointPosition;
    config2(2).JointPosition=configSoln(idx,2).JointPosition;
    config2(3).JointPosition=configSoln(idx,3).JointPosition;
    config2(4).JointPosition=configSoln(idx,4).JointPosition;
    show(robot2,config2, 'PreservePlot', false,'Frames','off');
    hold on
    if idx==1
        fnplt(traj,'r',2);
        plot3(wayPoints(:,1),wayPoints(:,2),wayPoints(:,3),'.','MarkerSize',40,  'MarkerEdgeColor','k');
    end
    pause(0.1)
end
hold off

%% Joint Command Matrix
% Initializes a matrix to hold joint commands
JointCommandsRad=zeros(size(eePos,2),numJoints);
wayPoints=wayPoints';

% Fills the joint command matrix with joint positions from the inverse kinematics solution
for i = 1:size(eePos,2)
    JointCommandsRad(i,1)=configSoln(i,1).JointPosition;
    JointCommandsRad(i,2)=configSoln(i,2).JointPosition;
    JointCommandsRad(i,3)=configSoln(i,3).JointPosition;
    JointCommandsRad(i,4)=configSoln(i,4).JointPosition;
end

% Converts joint commands to degrees
JointCommandsRad=[JointCommandsRad(1,:); JointCommandsRad];
JointCommandsDeg=JointCommandsRad*180/pi;

%% Command Signals to Servo Joints
% Defines the total time and the step size for the trajectory
tot=20;
step=tot/totalPoints;
time=0:step:tot;

% Creates a new figure and sets its properties
figure

% Plots the command signal for the base joint
base.time=time';
base.signals.values=JointCommandsDeg(:,1);
subplot(2,2,1);
plot(time,JointCommandsDeg(:,1)', 'linewidth', 3.5, 'Color', 'r');
title('Base Motor Signal', 'FontSize', 30);
xlabel('Time [s]', 'FontSize', 30);
ylabel('Joint Angle [deg]', 'FontSize', 30);
grid on;
bLim=80;
tLim=100;
% Set y-axis limits
ylim([bLim tLim]);
% Set the number of steps/values on the y-axis
set(gca, 'YTick', bLim:((tLim-bLim)/2):tLim);
% Change the color of the x and y axis lines to black and line width to 1.2
set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)

% Change the font size of the x and y tick labels
set(gca, 'FontSize', 20)

% Make the grid more transparent
set(gca, 'GridAlpha', 0.02, 'MinorGridAlpha', 0.02);
% Plots the command signal for the shoulder joint
shoulder.time=time';
shoulder.signals.values=JointCommandsDeg(:,2);
subplot(2,2,2);
plot(time,JointCommandsDeg(:,2)', 'linewidth', 3.5, 'Color', 'r');
title('Shoulder Motor Signal', 'FontSize', 30);
xlabel('Time [s]', 'FontSize', 30);
ylabel('Joint Angle [deg]', 'FontSize', 30);
grid on;
bLim=20;
tLim=110;
% Set y-axis limits
ylim([bLim tLim]);
% Set the number of steps/values on the y-axis
set(gca, 'YTick', bLim:((tLim-bLim)/6):tLim);
% Change the color of the x and y axis lines to black and line width to 1.2
set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)

% Change the font size of the x and y tick labels
set(gca, 'FontSize', 20)

% Make the grid more transparent
set(gca, 'GridAlpha', 0.02, 'MinorGridAlpha', 0.02);
% Plots the command signal for the elbow joint
elbow.time=time';
elbow.signals.values=JointCommandsDeg(:,3);
subplot(2,2,3);
plot(time,JointCommandsDeg(:,3)', 'linewidth', 3.5, 'Color', 'r');
title('Elbow Motor Signal', 'FontSize', 30);
xlabel('Time [s]', 'FontSize', 30);
ylabel('Joint Angle [deg]', 'FontSize', 30);
grid on;
bLim=0;
tLim=60;
% Set y-axis limits
ylim([bLim tLim]);
% Set the number of steps/values on the y-axis
set(gca, 'YTick', bLim:((tLim-bLim)/6):tLim);
% Change the color of the x and y axis lines to black and line width to 1.2
set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)

% Change the font size of the x and y tick labels
set(gca, 'FontSize', 20)

% Make the grid more transparent
set(gca, 'GridAlpha', 0.02, 'MinorGridAlpha', 0.02);
% Plots the command signal for the wrist joint
wrist.time=time';
wrist.signals.values=JointCommandsDeg(:,4);
subplot(2,2,4);
plot(time,JointCommandsDeg(:,4)', 'linewidth', 3.5, 'Color', 'r');
title('Wrist Motor Signal', 'FontSize', 30);
xlabel('Time [s]', 'FontSize', 30);
ylabel('Joint Angle [deg]', 'FontSize', 30);
grid on;
bLim=10;
tLim=70;
% Set y-axis limits
ylim([bLim tLim]);
% Set the number of steps/values on the y-axis
set(gca, 'YTick', bLim:((tLim-bLim)/6):tLim);

% Change the color of the x and y axis lines to black and line width to 1.2
set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)

% Change the font size of the x and y tick labels
set(gca, 'FontSize', 20)

% Make the grid more transparent
set(gca, 'GridAlpha', 0.02, 'MinorGridAlpha', 0.02);