<<<<<<< HEAD
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
subplot(1,3,1);show(robot);

homeConfig = robot.homeConfiguration;

%Finds the transformation between the end effector and robot base frame. 
%This transformation matrix converts points in the end_effector space to
%base robot space when robot is at "homeConfig" configuration. 
tformHome = getTransform(robot,homeConfig,end_effector);


%robot in a random configuration
randConfig = robot.randomConfiguration;

subplot(1,3,2);show(robot,randConfig);
tform = getTransform(robot,randConfig,end_effector);


weights = [0.25 0.25 0.25 1 1 1];

ik = robotics.InverseKinematics('RigidBodyTree',robot);

[configSoln,solnInfo] = ik(end_effector,tformHome,weights,randConfig);

subplot(1,3,3);show(robot,configSoln);






=======
clc;
clear;

%hello

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
>>>>>>> 6b4b68da9c61b04e1cfac4d050d411104244ff78
