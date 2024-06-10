% Conexión a Coppelia 

clc; clear;
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1', 19000, true, true, 5000, 5);

%% 

% Get Handles

[err, left_Motor] = vrep.simxGetObjectHandle(id,'./leftMotorPID',vrep.simx_opmode_blocking);
[err, right_Motor] = vrep.simxGetObjectHandle(id,'./rightMotorPID',vrep.simx_opmode_blocking);
[err, path] = vrep.simxGetObjectHandle(id,'./Path',vrep.simx_opmode_blocking);
[err, ref_point] = vrep.simxGetObjectHandle(id,'./ref_point',vrep.simx_opmode_blocking);
[err, robot_pose] = vrep.simxGetObjectHandle(id,'./robot_pose',vrep.simx_opmode_blocking);

while 1
    [err, position] = vrep.simxGetObjectPosition(id, robot_pose,-1,vrep.simx_opmode_streaming);
    [err, orientation] = vrep.simxGetObjectOrientation(id, robot_pose,-1,vrep.simx_opmode_streaming);
    loc_pose = [position(1), position(2), orientation(3)];

    [err] = vrep.simxSetJointTargetVelocity(id,left_Motor,0.3,vrep.simx_opmode_blocking);
    [err] = vrep.simxSetJointTargetVelocity(id,right_Motor,0.3,vrep.simx_opmode_blocking);
    disp(loc_pose)
end

% pose = updateRobotPose();
% [err] = vrep.simxSetJointTargetVelocity(id,left_Motor,0.3,vrep.simx_opmode_blocking);
% [err] = vrep.simxSetJointTargetVelocity(id,right_Motor,0.1,vrep.simx_opmode_blocking);
% 
% pause(5)
% 
% [err] = vrep.simxSetJointTargetVelocity(id,left_Motor,0.1,vrep.simx_opmode_blocking);
% [err] = vrep.simxSetJointTargetVelocity(id,right_Motor,0.2,vrep.simx_opmode_blocking);





%% Functions

function [vtraj,ptraj] = getTrajectoryPoint()
    [err, position] = vrep.simxGetObjectPosition(id, ref_point,-1,vrep.simx_opmode_streaming);
    [err, orientation] = vrep.simxGetObjectOrientation(id, ref_point,-1,vrep.simx_opmode_streaming);
    [err, linear_vel, angular_vel] = vrep.simxGetObjectVelocity(id, ref_point,vrep.simx_opmode_streaming);
    
    if orientation(3)>0
        ptraj = [position(1), position(2), orientation(2) - pi/2];
    else
        ptraj = [position(1), position(2), pi/2-orientation(2)];
    end

    vtraj = [linear_vel(1), linear_vel(2), angular_vel(3) ];
end


function [loc_pose] = updateRobotPose()
    [err, position] = vrep.simxGetObjectPosition(id, robot_pose,-1,vrep.simx_opmode_streaming);
    [err, orientation] = vrep.simxGetObjectOrientation(id, robot_pose,-1,vrep.simx_opmode_streaming);
    loc_pose = [position(1), position(2), orientation(3)];
end
