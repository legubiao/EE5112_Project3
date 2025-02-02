%Parameters
nDiscretize = 20;           % number of discretized waypoint 路径点的数量
nPaths = 20;                % number of sample paths 每次采样的粒子的数量
convergenceThreshold = 0.1; % convergence threshhold 收敛阈值

% Initial guess of joint angles theta is just linear interpolation of q0
% and qT
q0 = currentRobotJConfig;
qT = finalRobotJConfig;
numJoints = length(q0);
theta=zeros(numJoints, nDiscretize);
for k=1:length(q0)
    theta(k,:) = linspace(q0(k), qT(k), nDiscretize);
end
theta_animation = {};

% by default, it loads the robot with the structure data format
robot_struct = loadrobot(robot_name); 

% store sampled paths
theta_samples = cell(1,nPaths);

%% for calculating the acceleration of theta
% Precompute
A_k = eye(nDiscretize - 1, nDiscretize - 1);
A = -2 * eye(nDiscretize, nDiscretize);
A(1:nDiscretize - 1, 2:nDiscretize) = A(1:nDiscretize - 1, 2:nDiscretize) + A_k;
A(2:nDiscretize, 1:nDiscretize - 1) = A(2:nDiscretize, 1:nDiscretize - 1) + A_k;
A = A(:, 2:end-1); 
R = A' * A;
Rinv = inv(R);
M = 1 / nDiscretize * Rinv ./ max(Rinv, [], 1); % normalized by each column, no longer symmetric
Rinv = 1.5*Rinv/sum(sum(Rinv)); % normalized R inverse, so that the sample is still within the voxel world


%%
%Planner
Q_time = [];   % Trajectory cost Q(theta), t-vector
RAR_time = [];

[~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);
QthetaOld = 0;
iter=0;
while abs(Qtheta - QthetaOld) > convergenceThreshold
    iter=iter+1;
    % overall cost: Qtheta
    QthetaOld = Qtheta;
    % use tic and toc for printing out the running time
    tic
    %% TODO: Complete the following code. The needed functions are already given or partially given in the folder.
    %% TODO: Sample noisy trajectories
    [theta_paths, em] = stompSamples(nPaths,Rinv,theta);

    %% TODO: Calculate Local trajectory cost for each sampled trajectory
    % variable declaration (holder for the cost):
    Stheta = zeros(nPaths, nDiscretize);
    for i=1:nPaths
        [Stheta(i,:), ~] = stompTrajCost(robot_struct, theta_paths{i}, R, voxel_world);
    end
    
    %% TODO: Given the local traj cost, update local trajectory probability
    trajProb = stompUpdateProb(Stheta);
    
    %% TODO: Compute delta theta (aka gradient estimator, the improvement of the delta)
    dtheta = stompDTheta(trajProb, em);

    %% TODO: Compute the cost of the new trajectory
    [theta, dtheta_smoothed] = stompUpdateTheta(theta, dtheta, M);
    [~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);
    theta_animation{end+1} = theta;

    toc

    Q_time = [Q_time Qtheta];
    % control cost
    RAR = 1/2 * sum(sum(theta(:, 2:nDiscretize-1) * R * theta(:, 2:nDiscretize-1)'));
    RAR_time = [RAR_time RAR];
    Qtheta % display overall cost
    RAR  % display control cost

    % Set the stopping iteration criteria:
    if iter > 50 
        disp('Maximum iteration (50) has reached.')
        break
    end

    if sum(dtheta_smoothed,'all') == 0
    disp('Estimated gradient is 0 and Theta is not updated: there could be no obstacle at all')
    break
    end

end

disp('STOMP Finished.');






%% check collision
inCollision = false(nDiscretize, 1); % initialization the collision status vector
worldCollisionPairIdx = cell(nDiscretize,1); % Initialization: Provide the bodies that are in collision

for i = 1:nDiscretize

    [inCollision(i),sepDist] = checkCollision(robot,theta(:,i),world,"IgnoreSelfCollision","on","Exhaustive","on");


    [bodyIdx,worldCollisionObjIdx] = find(isnan(sepDist)); % Find collision pairs
    worldCollidingPairs = [bodyIdx,worldCollisionObjIdx];
    worldCollisionPairIdx{i} = worldCollidingPairs;

end
isTrajectoryInCollision = any(inCollision)


%% Plot training iteration process
enableVideoTraining = 1;



v = VideoWriter('KukaIiwa7_Training.avi');
v.FrameRate = 15;
open(v);

htext = text(-0.2,0.6,0.7,'Iteration = 0','HorizontalAlignment','left','FontSize',14);

if enableVideoTraining == 1
    theta_animation_tmp = theta_animation(~cellfun('isempty',theta_animation));
    nTraining = length(theta_animation_tmp);
    for k=0:5:nTraining
       
        UpdatedText = ['Iteration = ',num2str(k)];
        set(htext,'String',UpdatedText)

        if k+1 > length(theta_animation_tmp)
            theta_tmp = theta_animation_tmp{k};
        else
            theta_tmp = theta_animation_tmp{k+1};
        end

        for t=1:size(theta_tmp,2)
            show(robot, theta_tmp(:,t),'PreservePlot', false, 'Frames', 'on');
            %             drawnow;
            frame = getframe(gcf);
            writeVideo(v,frame);
%             pause(1/15);
            %     pause;
        end
        pause(1/15);
    end
end
close(v);



%% Plot path
enableVideo = 1;
if enableVideo == 1
    v = VideoWriter('KukaIiwa7_wEEConY3.avi');
    v.FrameRate =2;
    open(v);

    for t=1:size(theta,2)
        show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'on');
        drawnow;
        frame = getframe(gcf);
        writeVideo(v,frame);
        pause(5/20);
        %     pause;
    end
    close(v);
end
%%
displayAnimation = 1;
if displayAnimation
    for t=1:size(theta,2)
        show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'on');
        drawnow;
        pause(5/20);
        %     pause;
    end
end



%% save data
filename = ['Theta_nDisc', num2str(nDiscretize),'_nPaths_', num2str(nPaths), '.mat'];
save(filename,'theta')

