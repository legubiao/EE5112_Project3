%% forward kinematics
% INPUT: 
%       robot_struct: the Matlab robot structure object
%       theta: the joints rotation angles
% OUTPUT:
%       X: Joints' positions in the world frame
%       T: Homogeneous Transformation from the Joint frame to the base
%       frame
function [X, T] = updateJointsWorldPosition(robot_struct, theta)

% In this sample code, we directly call the MATLAB built-in function getTransform
% to calculate the forward kinemetics
    

% Update the robot configuration structure used by Matlab
% Refer to: https://www.mathworks.com/matlabcentral/answers/31273-how-to-update-struct-array-fields-with-mutiple-values
theta_cell = num2cell(theta);
% Because the getTranform() function can only takes in structure array
% robot Configuration, we have to construct it first by copying the structure from the homeConfiguration
% robot_struct = loadrobot(robot_name); % by default, it loads the robot with structure data format
tConfiguration= robot_struct.homeConfiguration; % copy the home configuration struct
[tConfiguration.JointPosition]= theta_cell{:}; % update the Joint position using theta
% get the number of joints
nJoints = length(theta);
T = cell(1,nJoints);
X = zeros(nJoints, 4); 

first_joint = 1;
for i=1:robot_struct.NumBodies
   joint = robot_struct.Bodies{1,i}.Joint;
   if (joint.Type ~= "fixed")
       first_joint = i;
       break
   end
end

usePoE = true;
comparePoE = false;

if usePoE
    M = eye(4);
    Slist = zeros(6,nJoints);
    Mlist = cell(1,nJoints);
    if (first_joint > 1)
        for k = 1:first_joint-1
            currentJoint = robot_struct.Bodies{1,k}.Joint;
            M = M * currentJoint.JointToParentTransform;
        end
    end

    for k=1:nJoints
        currentJoint = robot_struct.Bodies{1,k + first_joint - 1}.Joint;
        M = M * currentJoint.JointToParentTransform;
        Mlist{k} = M;
        omega = M(1:3,1:3) * currentJoint.JointAxis';
        v = cross(M(1:3,4),omega);
        Slist(1:3,k) = omega;
        Slist(4:6,k) = v;
    end
    
    T_temp = M;
    for i = size(theta): -1: 1
        T_temp = MatrixExp6(VecTose3(Slist(:, i) * theta(i))) * T_temp;
        T{i} = T_temp;
    end
    
    for i = 1: nJoints -1
        T{i} = T_temp / (Mlist{i} \ T{i+1});
        X(i,:) =  T{i}(1:4,4)';
    end

    T{nJoints} = T_temp;
    X(nJoints,:) =  T{nJoints}(1:4,4)';

    if comparePoE
        T_compare = cell(1,nJoints);
        for k=1:nJoints
            T_compare{k} = getTransform(robot_struct, tConfiguration, robot_struct.BodyNames{k+first_joint - 1});
        end
    end

    
else
    for k=1:nJoints
        % get the homegeneous transformation from kth joint's frame to the
        % base frame
        % Use the Matlab built-in function getTransfrom to obtain the pose T
        % getTransform can only takes in structure array Configuration
        %% TODO:
        % 使用matlab自带的getTransform函数，直接根据机器人结构和各个关节的角度，计算各个frame的T矩阵
        T{k}=getTransform(robot_struct, tConfiguration, robot_struct.BodyNames{k+first_joint - 1});
        % Get joint's world coordinates
        X(k,:) =  T{k}(1:4,4)';
    end
end
    
end