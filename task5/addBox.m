function [Obs, binary_world] = addBox(binary_world, Env_size, voxel_size,  lbox, box_center)
%ADDBOX 此处显示有关此函数的摘要
%   lbox: the length of the box
%   center: the 


Obs = collisionBox(lbox,lbox,lbox);  
Obs.Pose = trvec2tform(box_center);   

%% voxelize the box obstacles
cube_metric = [box_center-lbox/2;
                lbox,lbox,lbox]; % [xmin, ymin, zmin] for 1st row, xyz-lengths for 2nd row
% range (lower and upper limits) of the cube voxel
% The number inside ceil is always positive
cube_voxel = [ceil((cube_metric(1, :)-Env_size(1,:))./voxel_size); ...
    ceil((cube_metric(1, :) + cube_metric(2, :)-Env_size(1,:))./voxel_size)];

% Update the cube occupancy in the voxel world
% First generate the 3D grid coordinates (x,y,z subcripts) of cube in the voxel world
[xc, yc, zc] = meshgrid(cube_voxel(1, 1):cube_voxel(2, 1), cube_voxel(1, 2):cube_voxel(2, 2), cube_voxel(1, 3):cube_voxel(2, 3));
% Set the corresponding voxel occupancy to 1, meaning occupied
binary_world(sub2ind([Env_size(2, 1) / voxel_size(1), Env_size(2, 2) / voxel_size(2), Env_size(2, 3) / voxel_size(3)], xc, yc, zc)) = 1;

end

