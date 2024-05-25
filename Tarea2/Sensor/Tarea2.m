close all;
clear;
clc;

%Conectarse a Coppelia
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1', 19000, true, true, 5000, 5);

%Obtener Handle de visor
%https://manual.coppeliarobotics.com/en/remoteApiFunctionsMatlab.htm
[err, camhandle] = vrep.simxGetObjectHandle(id,'fast3DLaserScanner', vrep.simx_opmode_oneshot_wait);

%Obtener Handle de motores
[err, left_Motor] = vrep.simxGetObjectHandle(id,'./leftMotor',vrep.simx_opmode_blocking);
[err, right_Motor] = vrep.simxGetObjectHandle(id,'./rightMotor',vrep.simx_opmode_blocking);

%%
%Dar velocidad a motores
[err] = vrep.simxSetJointTargetVelocity(id,left_Motor,0.1,vrep.simx_opmode_blocking);
[err] = vrep.simxSetJointTargetVelocity(id,right_Motor,0.1,vrep.simx_opmode_blocking);
iter = 1000;

for n=linspace(1, iter, iter)
    [returnCode, outInt, OutFloat, OutStr, OutBuf] = vrep.simxCallScriptFunction(id, 'fast3DLaserScanner', vrep.sim_scripttype_childscript, 'sysCall_sensing', [], [], '', [], vrep.simx_opmode_blocking);
    M = length(OutFloat) / 3;
    B = reshape(OutFloat, [3, M]);
    x = B(1, :); y = B(2, :); z = B(3, :);
    figure(1); plot3(x, y, z, '.', "LineWidth", 0.5); xlabel('X');
    ylim([-1 1]); xlim([0 1]);
    title("iteracion: "+int2str(n));
    ylabel('Y'); zlabel('Z');
    view(-90, 0);
    grid on;
end