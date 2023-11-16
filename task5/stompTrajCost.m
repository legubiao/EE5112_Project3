% Given a trajectory, calculate its cost
function [Stheta, Qtheta] = stompTrajCost(robot_struct, theta,  R, voxel_world)
% Compute the local trajectory cost at each discretization theta point, as 
% well as the overall trajecotry cost (the Qtheta)

% Costi = stompCompute_Cost(robot, theta, Env);
% Compute the cost of all discretized points on one trajectory
[~, nDiscretize] = size(theta);
% Obstacle cost 
qo_cost = zeros(1, nDiscretize);
% Constraint costs
qc_cost = zeros(1, nDiscretize);

% Get the coordinates of joints in World frame 
[X, ~] = updateJointsWorldPosition(robot_struct, theta(:, 1));
% Construct the spheres around the robot manipulator for collision
% avoidance
[sphere_centers,radi] = stompRobotSphere(X);
% Initial velocity at the sphere centers around the manipulator is 0
vel = zeros(length(sphere_centers), 1);
qo_cost(1) = stompObstacleCost(sphere_centers,radi, voxel_world, vel);

for i = 2 : nDiscretize
    sphere_centers_prev = sphere_centers;
    % Calculate the kinematics of the manipulator, given the
    % configuration theta values at different time (i=2:nDiscretize)
    [X, T] = updateJointsWorldPosition(robot_struct, theta(:, i));
    [sphere_centers, radi] = stompRobotSphere(X);
    % xb: 3D workspace position of sphere b at the current time
    % Approximate the speed (xb_dot) using the finite difference of the current and
    % the previous position
    vel = vecnorm(sphere_centers_prev - sphere_centers,2,2);
    qo_cost(i) = stompObstacleCost(sphere_centers,radi, voxel_world, vel);
    
    %% TODO: Define your qc_cost to add constraint on the end-effector
    end_constrain=true;
    if end_constrain==true
        desired_orientation = [0, 0, -1]; % 期望的直立方向
        % 计算末端执行器当前方向
        current_orientation = calculateEndEffectorOrientation(T);
        % 计算方向偏差
        angle_deviation = acos(dot(desired_orientation, current_orientation) / ...
        (norm(desired_orientation) * norm(current_orientation)));
        % 将偏差角度的平方作为额外成本
        weight_factor=1000;
        qc_cost(i) = weight_factor * angle_deviation^2;
    else
        qc_cost = 0;
    end
end

%% Local trajectory cost: you need to specify the relative weights between different costs
Stheta = 1000*qo_cost + qc_cost;

% sum over time and add the smoothness cost
theta = theta(:, 2:end-1);
Qtheta = sum(Stheta) + 1/2 * sum(theta * R * theta', "all");

end

function current_orientation = calculateEndEffectorOrientation(T)
    % 假设 T 是一个从每个关节框架到基础框架的齐次变换数组
    % 末端执行器的变换矩阵是数组的最后一个元素
    end_effector_transform = T{end};

    % 从变换矩阵中提取 y 轴的方向
    % MATLAB 中的变换矩阵是 4x4 的，其中前三列分别是 x, y, z 轴的方向，第四列是位置
    % 我们需要第二列，即 y 轴的方向
    current_orientation = end_effector_transform(1:3, 3);
end
    
    