%% Import the Robot and add the Virtual End Effector
clear
clc

robot = importrobot('Tinkerkit_model/tinkerkit4DOF.urdf');
robot2 = importrobot('Tinkerkit_model/tinkerkit.urdf');
numJoints = numel(homeConfiguration(robot));

eeOffset=0.16;
eeBody=robotics.RigidBody('end_effector');
setFixedTransform(eeBody.Joint, trvec2tform([0 0 eeOffset]));
addBody(robot,eeBody,'link4');

clc

%% Set HOME position
config = homeConfiguration(robot);
config2 = homeConfiguration(robot2);

config(1).JointPosition=pi/2;
config(2).JointPosition=pi/2;
config(3).JointPosition=pi/4;
config(4).JointPosition=pi/4;

show(robot, config);
title('Braccio: HOME Configuration')
axis([-0.3 0.3 -0.3 0.3 -0.1 0.5]);
hold on

%% Detect Points to be Reached and Trajectory Tracking
% Waypoints
adStartPoint    = [0.32 0 0.1];
adBallPoint     = [0.32 0 -0.05];
adPoint         = [0.25 0 0.1];
adGoalPoint     = [0.20 0 0.2];
adEndPoint      = [0.20 0 -0.05];
                   %x y z
adWaypoints     = [adStartPoint;
                    adBallPoint;
                    adPoint;
                    adGoalPoint;
                    adEndPoint];
wayPoints=adWaypoints
plot3(wayPoints(:,1),wayPoints(:,2),wayPoints(:,3),'.','MarkerSize',40,  'MarkerEdgeColor','k'); %punti di passaggio dell'EE
hold on

traj=cscvn(wayPoints');
fnplt(traj,'r',2);
grid on
hold off

%% Inverse Kinematics Trajectory
ik = robotics.InverseKinematics('RigidBodyTree',robot);
ik.SolverAlgorithm = 'LevenbergMarquardt';
weights = [0 0 0 1 1 1];
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
JointCommandsRad=zeros(size(eePos,2),numJoints);
wayPoints=wayPoints';

for i = 1:size(eePos,2)
    JointCommandsRad(i,1)=configSoln(i,1).JointPosition;
    JointCommandsRad(i,2)=configSoln(i,2).JointPosition;
    JointCommandsRad(i,3)=configSoln(i,3).JointPosition;
    JointCommandsRad(i,4)=configSoln(i,4).JointPosition;
end

JointCommandsRad=[JointCommandsRad(1,:); JointCommandsRad];
JointCommandsDeg=JointCommandsRad*180/pi;

%% Command Signals to Servo Joints
tot=20;
step=tot/totalPoints;
time=0:step:tot;

figure
base.time=time';
base.signals.values=JointCommandsDeg(:,1);
subplot(2,2,1);
plot(time,JointCommandsDeg(:,1)');
title('Base Motor Signal')
grid on

shoulder.time=time';
shoulder.signals.values=JointCommandsDeg(:,2);
subplot(2,2,2);
plot(time,JointCommandsDeg(:,2)');
title('Shoulder Motor Signal')
grid on

elbow.time=time';
elbow.signals.values=JointCommandsDeg(:,3);
subplot(2,2,3);
plot(time,JointCommandsDeg(:,3)');
title('Elbow Motor Signal')
grid on

wrist.time=time';
wrist.signals.values=JointCommandsDeg(:,4);
subplot(2,2,4);
plot(time,JointCommandsDeg(:,4)');
title('Wrist Motor Signal')
grid on
