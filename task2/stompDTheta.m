% dtheta: estimated gradient
% em: 1 by nJoints cell, each cell is nSamples by nDiscretize matrix
function dtheta = stompDTheta(trajProb, em)

nJoints = length(em);
nDiscretize = size(trajProb, 2);
% variable declaration
dtheta = zeros(nJoints, nDiscretize);

%% TODO: iterate over all joints to compute dtheta: (complete your code according to the STOMP algorithm)
for i = 1:nJoints
    for t = 1:nDiscretize
        dtheta(i, t) = sum(trajProb(i, :) * em{i}(:, t));
    end
end
