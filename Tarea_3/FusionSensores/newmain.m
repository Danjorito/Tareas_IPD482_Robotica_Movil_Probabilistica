%%
% Inicializacion de la API
clc; clear;

vrep = remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1', 19000, true, true, 5000, 5);

%%
% Obtener handle y primera corrida de obtener posicion
%https://manual.coppeliarobotics.com/en/remoteApiFunctionsMatlab.html
[err, camhandle] = vrep.simxGetObjectHandle(id,'./VelodyneVPL16', vrep.simx_opmode_oneshot_wait);
[err, posehandle] = vrep.simxGetObjectHandle(id,'./Robot_Pose', vrep.simx_opmode_oneshot_wait);
[err, angle] = vrep.simxGetObjectOrientation(id, posehandle, -1, vrep.simx_opmode_streaming);
[err, pos] = vrep.simxGetObjectPosition(id, posehandle, -1, vrep.simx_opmode_streaming);
[err, GPShandle] = vrep.simxGetObjectHandle(id,'./GPS', vrep.simx_opmode_oneshot_wait);
[err, Accelhandle] = vrep.simxGetObjectHandle(id,'./Accelerometer', vrep.simx_opmode_oneshot_wait);
[err, Gyrohandle] = vrep.simxGetObjectHandle(id,'./GyroSensor', vrep.simx_opmode_oneshot_wait);

[returnCode, GPSValuex] = vrep.simxGetFloatSignal(id,'datosGPSx', vrep.simx_opmode_streaming);
[returnCode, GPSValuey] = vrep.simxGetFloatSignal(id,'datosGPSy', vrep.simx_opmode_streaming);
[returnCode, GPSValuez] = vrep.simxGetFloatSignal(id,'datosGPSz', vrep.simx_opmode_streaming);

[returnCode, AccelValuex] = vrep.simxGetFloatSignal(id,'datosAccelx', vrep.simx_opmode_streaming);
[returnCode, AccelValuey] = vrep.simxGetFloatSignal(id,'datosAccely', vrep.simx_opmode_streaming);
[returnCode, AccelValuez] = vrep.simxGetFloatSignal(id,'datosAccelz', vrep.simx_opmode_streaming);

[returnCode, GyroValuex] = vrep.simxGetFloatSignal(id,'datosGyrox', vrep.simx_opmode_streaming);
[returnCode, GyroValuey] = vrep.simxGetFloatSignal(id,'datosGyroy', vrep.simx_opmode_streaming);
[returnCode, GyroValuez] = vrep.simxGetFloatSignal(id,'datosGyroz', vrep.simx_opmode_streaming);

angle_robot = angle(3)*180/pi;

%%
% Sensor GPS

close all; clc;

[returnCode, GPSValuex] = vrep.simxGetFloatSignal(id,'datosGPSx', vrep.simx_opmode_buffer);
[returnCode, GPSValuey] = vrep.simxGetFloatSignal(id,'datosGPSy', vrep.simx_opmode_buffer);
[returnCode, GPSValuez] = vrep.simxGetFloatSignal(id,'datosGPSz', vrep.simx_opmode_buffer);

%%
% Sensor Accel

close all; clc;

[returnCode, AccelValuex] = vrep.simxGetFloatSignal(id,'datosAccelx', vrep.simx_opmode_buffer);
[returnCode, AccelValuey] = vrep.simxGetFloatSignal(id,'datosAccely', vrep.simx_opmode_buffer);
[returnCode, AccelValuez] = vrep.simxGetFloatSignal(id,'datosAccelz', vrep.simx_opmode_buffer);

%%
% Sensor Gyro

close all; clc;

[returnCode, GyroValuex] = vrep.simxGetFloatSignal(id,'datosGyrox', vrep.simx_opmode_buffer);
[returnCode, GyroValuey] = vrep.simxGetFloatSignal(id,'datosGyroy', vrep.simx_opmode_buffer);
[returnCode, GyroValuez] = vrep.simxGetFloatSignal(id,'datosGyroz', vrep.simx_opmode_buffer);

%%
% velocidad
[err, left_Motor] = vrep.simxGetObjectHandle(id,'./leftMotor',vrep.simx_opmode_blocking);
[err, right_Motor] = vrep.simxGetObjectHandle(id,'./rightMotor',vrep.simx_opmode_blocking);

for n = linspace(1, 16*pi, 100)
    [err] = vrep.simxSetJointTargetVelocity(id,left_Motor, 5*cos(n),vrep.simx_opmode_blocking);
    [err] = vrep.simxSetJointTargetVelocity(id,right_Motor,5*cos(n),vrep.simx_opmode_blocking);
end

%%
% Parar

[err, left_Motor] = vrep.simxGetObjectHandle(id,'./leftMotor',vrep.simx_opmode_blocking);
[err, right_Motor] = vrep.simxGetObjectHandle(id,'./rightMotor',vrep.simx_opmode_blocking);

[err] = vrep.simxSetJointTargetVelocity(id,left_Motor, 0.1,vrep.simx_opmode_blocking);
[err] = vrep.simxSetJointTargetVelocity(id,right_Motor,0.5,vrep.simx_opmode_blocking);