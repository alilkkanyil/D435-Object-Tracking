clc;
clear;


end_effector = 'wrist_link';
base_link = 'base_link';

robot = importrobot("/Users/sdinc/Documents/GitHub/interbotix_ros_arms/interbotix_descriptions/urdf/rx150.urdf");

subplot(1,3,1);show(robot);


homeConfig = robot.homeConfiguration;
tformHome = getTransform(robot,homeConfig,end_effector,base_link);


randConfig = robot.randomConfiguration;
tform = getTransform(robot,randConfig,end_effector,base_link);

subplot(1,3,2);show(robot,randConfig);

weights = [0.25 0.25 0.25 1 1 1];

ik = robotics.InverseKinematics(robot);

[configSoln,solnInfo] = ik(end_effector,tform,weights,homeConfig);

subplot(1,3,3);show(robot,configSoln);