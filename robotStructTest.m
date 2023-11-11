% 机械臂导入研究
clc;
clear;
close all;
addpath('mr')


robot_name = 'kinovaGen3';
robot =  loadrobot(robot_name, 'DataFormat', 'column');
showdetails(robot)

nJoints = 4;

show(robot);

% 计算在初始位置下，从base到End的T矩阵
M = eye(4);
Slist = zeros(6,nJoints);
for k=1:nJoints
    currentJoint = robot.Bodies{1,k}.Joint;
    M = M * currentJoint.JointToParentTransform;
    omega = M(1:3,1:3) * currentJoint.JointAxis';
    v = -cross(omega,M(1:3,4));
    Slist(1:3,k) = omega;
    Slist(4:6,k) = v;
end

configuration = robot.homeConfiguration;

FKinSpace(M,Slist,configuration(1:nJoints))
getTransform(robot, configuration, robot.BodyNames{nJoints})