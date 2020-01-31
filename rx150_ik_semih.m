clc;
clear;

if ismac
    urdfFile = "/Users/sdinc/Documents/GitHub/interbotix_ros_arms/interbotix_descriptions/urdf/rx150.urdf";
elseif isunix
    urdfFile = "/Users/sdinc/Documents/GitHub/interbotix_ros_arms/interbotix_descriptions/urdf/rx150.urdf";
elseif ispc
    urdfFile = "C:/Users/sdinc/Documents/GitHub/interbotix_ros_arms/interbotix_descriptions/urdf/rx150.urdf";
else
    disp('Platform not supported')
end

end_effector = 'wrist_link';
base_link = 'base_link';

%robot in the home position
robot = importrobot(urdfFile);

%Create ik model for given robot
ik = robotics.InverseKinematics('RigidBodyTree',robot);

homeConfig = robot.homeConfiguration;
myConfig = homeConfig; 

myConfig(1).JointPosition = deg2rad(0);
myConfig(2).JointPosition = deg2rad(0);
myConfig(3).JointPosition = deg2rad(0);
myConfig(4).JointPosition = deg2rad(0);
myConfig(5).JointPosition = deg2rad(0);

tform1 = getTransform(robot,myConfig,end_effector)
rotm2eul(tform1(1:3,1:3))

%tform1(1:3,1:3) = eye(3);

weights = [0.025 0.025 0.025 0.1 0.1 0.1];
[confSol,~] = ik(end_effector,tform1,weights,homeConfig);
confSol.JointPosition

figure;show(robot,myConfig);

%%
%Finds the transformation between the end effector and robot base frame. 
%This transformation matrix converts points in the end_effector space to
%base robot space when robot is at "homeConfig" configuration. 

%getTransform Function works like a forward kinematics. It accepts a
%configuration (set of angles for each servo) and return the pose (4x4
%transformation matrix) of a joint in the robot. 
tformHome = getTransform(robot,homeConfig,end_effector);

targetPoseTranMat = tformHome; %I am using the original home pose
targetPoseTranMat(1,4) = 0.4; % and update only one dimension as the new target pose
targetPoseTranMat(3,4) = 0.2; % and update only one dimension as the new target pose

weights = [0.025 0.025 0.025 0.1 0.1 0.1];

%This function solves the inverse kinematics from homepose to target pose.
[configSoln,solnInfo] = ik(end_effector,targetPoseTranMat,weights,homeConfig);

%Prints the robot after application of the configuration.
% figure;show(robot,configSoln);


%%

% %robot in a random configuration
% randConfig = robot.randomConfiguration;
% 
% subplot(1,3,2);show(robot,randConfig);
% tform = getTransform(robot,randConfig,end_effector);
% 
% 
% weights = [0.25 0.25 0.25 1 1 1];
% 
% ik = robotics.InverseKinematics('RigidBodyTree',robot);
% 
% [configSoln,solnInfo] = ik(end_effector,tformHome,weights,randConfig);
% 
% subplot(1,3,3);show(robot,configSoln);