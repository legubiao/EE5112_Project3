%% forward kinematics
% INPUT: 
%       robot_struct: the Matlab robot structure object
%       theta: the joints rotation angles
% OUTPUT:
%       X: Joints' positions in the world frame
%       T: Homogeneous Transformation from the Joint frame to the base
%       frame
% 假设的末端执行器的“家”配置
function [X, T] = PoE_updateJointsWorldPosition(robot_struct, theta)

  % 初始化Slist和thetalist
    % Initialize the Slist matrix
    Slist = zeros(6, 9); % Assuming there are 7 active joints
    
    % Initialize the transformation matrix T as an identity matrix
    T = eye(4);
    
    % Joint information from the URDF file
    % Each row represents [r, p, y, x, y, z] for each joint
    joint_params = [
        0, 0, 0, 0, 0, 0.333;         % Joint 1
        -1.57079632679, 0, 0, 0, 0, 0; % Joint 2
        1.57079632679, 0, 0, 0, -0.316, 0; % Joint 3
        1.57079632679, 0, 0, 0.0825, 0, 0; % Joint 4
        -1.57079632679, 0, 0, -0.0825, 0.384, 0; % Joint 5
        1.57079632679, 0, 0, 0, 0, 0;   % Joint 6
        1.57079632679, 0, 0, 0.088, 0, 0;  % Joint 7
        0, 0, 0, 0, 0, 0.107; % Joint 8
        0, 0, -0.785398163397, 0, 0, 0; % Joint 9

    ];
    
    % Calculate the Slist matrix
    for i = 1:size(joint_params, 1)
        % Extract the joint parameters
        r = joint_params(i, 1);
        p = joint_params(i, 2);
        y = joint_params(i, 3);
        x = joint_params(i, 4);
        y_offset = joint_params(i, 5);
        z = joint_params(i, 6);
        
        % Calculate the homogeneous transformation matrix for the joint
        R = eul2tform([y, p, r]);
        T_joint = R * trvec2tform([x, y_offset, z]);
        
        % Update the transformation matrix T
        T = T * T_joint;
        
        % Extract the position vector from the transformation matrix T
        p = T(1:3, 4);
        
        % Calculate the screw axis for the joint
        Slist(:, i) = [0; 0; 1; -p(2); p(1); 0]; % For a revolute joint around z-axis
    end
    
    % Display the Slist matrix
    % disp(Slist);
    % 
    % 
    % Slist = zeros(6, 9);  % 7个活动关节和2个固定关节
    % 
    % % 关节1的信息
    % Slist(:, 1) = [0; 0; 1; 0; 0; 0.333];  % 关节1的螺旋轴
    % 
    % % 关节2的信息
    % Slist(:, 2) = [0; 0; 1; 0; 0; 0];  % 关节2的螺旋轴
    % 
    % % 关节3的信息
    % Slist(:, 3) = [0; 0; 1; 0; -0.316; 0];  % 关节3的螺旋轴
    % 
    % % 关节4的信息
    % Slist(:, 4) = [0; 0; 1; 0.0825; 0; 0];  % 关节4的螺旋轴
    % 
    % % 关节5的信息
    % Slist(:, 5) = [0; 0; 1; -0.0825; 0.384; 0];  % 关节5的螺旋轴
    % 
    % % 关节6的信息
    % Slist(:, 6) = [0; 0; 1; 0; 0; 0];  % 关节6的螺旋轴
    % 
    % % 关节7的信息
    % Slist(:, 7) = [0; 0; 1; 0.088; 0; 0];  % 关节7的螺旋轴
    % 
    % % 固定关节panda_joint8的信息
    % % 对于固定关节，螺旋轴为0，因为它们不会产生任何运动
    % Slist(:, 8) = zeros(6, 1);  % 关节8的螺旋轴
    % 
    % % panda_hand_joint的信息
    % % 手部为固定关节，同样螺旋轴为0
    % Slist(:, 9) = zeros(6, 1);  % 手部关节的螺旋轴
    % 初始化 M 矩阵为单位矩阵
    M = eye(4);
    
    % 通过每个关节的变换迭代
    % 注意：rpy 表示旋转的 roll, pitch, yaw，需要转换为 MATLAB 中的轴角表示
    % xyz 表示平移
    
    % panda_joint1 的变换
    M = M * axang2tform([0 0 1 0]) * trvec2tform([0 0 0.333]);
    
    % panda_joint2 的变换
    M = M * axang2tform([1 0 0 -pi/2]);
    
    % panda_joint3 的变换
    M = M * axang2tform([1 0 0 pi/2]) * trvec2tform([0 -0.316 0]);
    
    % panda_joint4 的变换
    M = M * axang2tform([1 0 0 pi/2]) * trvec2tform([0.0825 0 0]);
    
    % panda_joint5 的变换
    M = M * axang2tform([1 0 0 -pi/2]) * trvec2tform([-0.0825 0.384 0]);
    
    % panda_joint6 的变换
    M = M * axang2tform([1 0 0 pi/2]);
    
    % panda_joint7 的变换
    M = M * axang2tform([1 0 0 pi/2]) * trvec2tform([0.088 0 0]);
    
    % panda_joint8 的变换（固定关节）
    M = M * trvec2tform([0 0 0.107]);
    
    % panda_hand_joint 的变换（固定关节）
    M = M * axang2tform([0 0 1 -0.785398163397]);  % 这里假设 rpy 表示围绕 Z 轴的旋转
    
    % 输出最终的 M 矩阵
    % disp(M);

    T = FKinSpace(M, Slist, theta);

    % 计算关节的世界坐标
    X = zeros(length(theta), 4);
    for i = 1:length(theta)
        % 使用传入的theta计算每个关节i的世界位置
        T_i = FKinSpace(M, Slist(:, 1:i), theta(1:i));
        X(i, 1:3) = T_i(1:3, 4)';
        X(i, 4) = 1; % 齐次坐标的最后一项是 1
    end
    % disp(X)
    % theta_cell = num2cell(theta);
    % % Because the getTranform() function can only takes in structure array
    % % robot Configuration, we have to construct it first by copying the structure from the homeConfiguration
    % % robot_struct = loadrobot(robot_name); % by default, it loads the robot with structure data format
    % tConfiguration= robot_struct.homeConfiguration; % copy the home configuration struct
    % [tConfiguration.JointPosition]= theta_cell{:}; % update the Joint position using theta
    % % get the number of joints
    % nJoints = length(theta);
    % T = cell(1,nJoints);
    % X = zeros(nJoints, 4); 
    % 
    % for k=1:nJoints
    % % get the homegeneous transformation from kth joint's frame to the
    % % base frame
    % % Use the Matlab built-in function getTransfrom to obtain the pose T
    % % getTransform can only takes in structure array Configuration
    % %% TODO:
    % T{k}=getTransform(robot_struct, tConfiguration, robot_struct.Bodies{k}.Name);
    % % Get joint's world coordinates
    % X(k,1:3) = T{k}(1:3, end)';
    % X(k, 4) = 1;
    % end
    % disp(X(k, 4))
    % disp(X) 
end

function T = FKinSpace(M, Slist, thetalist)
    T = M;
    for i = size(thetalist, 1):-1:1
        T = MatrixExp6(VecTose3(Slist(:, i) * thetalist(i))) * T;
    end
end

function T = MatrixExp6(se3mat)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a se(3) representation of exponential coordinates.
% Returns a T matrix in SE(3) that is achieved by traveling along/about the 
% screw axis S for a distance theta from an initial configuration T = I.
% Example Input:
% 
% clear; clc;
% se3mat = [ 0,      0,       0,      0;
%          0,      0, -1.5708, 2.3562;
%          0, 1.5708,       0, 2.3562;
%          0,      0,       0,      0]
% T = MatrixExp6(se3mat)
% 
% Output:
% T =
%    1.0000         0         0         0
%         0    0.0000   -1.0000   -0.0000
%         0    1.0000    0.0000    3.0000
%         0         0         0    1.0000 

omgtheta = so3ToVec(se3mat(1: 3, 1: 3));
if NearZero(norm(omgtheta))
    T = [eye(3), se3mat(1: 3, 4); 0, 0, 0, 1];
else
    [omghat, theta] = AxisAng3(omgtheta);
    omgmat = se3mat(1: 3, 1: 3) / theta; 
    T = [MatrixExp3(se3mat(1: 3, 1: 3)), ...
         (eye(3) * theta + (1 - cos(theta)) * omgmat ...
          + (theta - sin(theta)) * omgmat * omgmat) ...
            * se3mat(1: 3, 4) / theta;
         0, 0, 0, 1];
end
end
function omg = so3ToVec(so3mat)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a 3x3 skew-symmetric matrix (an element of so(3)).
% Returns the corresponding 3-vector (angular velocity).
% Example Input: 
% 
% clear; clc;
% so3mat = [[0, -3, 2]; [3, 0, -1]; [-2, 1, 0]];
% omg = so3ToVec(so3mat)  
% 
% Output:
% omg =
%     1
%     2
%     3

omg = [so3mat(3, 2); so3mat(1, 3); so3mat(2, 1)];
end
function judge = NearZero(near)
% *** BASIC HELPER FUNCTIONS ***
% Takes a scalar.
% Checks if the scalar is small enough to be neglected.
% Example Input:
%  
% clear; clc;
% near = -1e-7;
% judge = NearZero(near)
% 
% Output:
% judge =
%     1

judge = norm(near) < 1e-6;
end
function [omghat, theta] = AxisAng3(expc3)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes A 3-vector of exponential coordinates for rotation.
% Returns the unit rotation axis omghat and the corresponding rotation 
% angle theta.
% Example Input:
% 
% clear; clc;
% expc3 = [1; 2; 3];
% [omghat, theta] = AxisAng3(expc3)  
% 
% Output:
% omghat =
%    0.2673
%    0.5345
%    0.8018
% theta =
%    3.7417

theta = norm(expc3);
omghat = expc3 / theta;
end
function  R = MatrixExp3(so3mat)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a 3x3 so(3) representation of exponential coordinates.
% Returns R in SO(3) that is achieved by rotating about omghat by theta 
% from an initial orientation R = I.
% Example Input:
% 
% clear; clc;
% so3mat = [[0, -3, 2]; [3, 0, -1]; [-2, 1, 0]];
% R = MatrixExp3(so3mat)  
% 
% Output:
% R =
%   -0.6949    0.7135    0.0893
%   -0.1920   -0.3038    0.9332
%    0.6930    0.6313    0.3481

omgtheta = so3ToVec(so3mat);
if NearZero(norm(omgtheta))
    R = eye(3);
else
    [omghat, theta] = AxisAng3(omgtheta);
    omgmat = so3mat / theta;
    R = eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
end
end
function se3mat = VecTose3(V)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a 6-vector (representing a spatial velocity).
% Returns the corresponding 4x4 se(3) matrix.
% Example Input:
% 
% clear; clc;
% V = [1; 2; 3; 4; 5; 6];
% se3mat = VecTose3(V)
% 
% Output:
% se3mat =
%     0    -3     2     4
%     3     0    -1     5
%    -2     1     0     6
%     0     0     0     0 

se3mat = [VecToso3(V(1: 3)), V(4: 6); 0, 0, 0, 0];
end
function so3mat = VecToso3(omg)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a 3-vector (angular velocity).
% Returns the skew symmetric matrix in so(3).
% Example Input:
% 
% clear; clc;
% omg = [1; 2; 3];
% so3mat = VecToso3(omg)
% 
% Output:
% so3mat =
%     0    -3     2
%     3     0    -1
%    -2     1     0

so3mat = [0, -omg(3), omg(2); omg(3), 0, -omg(1); -omg(2), omg(1), 0];
end