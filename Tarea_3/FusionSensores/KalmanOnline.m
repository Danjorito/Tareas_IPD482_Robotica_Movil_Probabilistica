%%
% Inicializacion de la API
clc; clear;

vrep = remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1', 19000, true, true, 5000, 5);

%[err, camhandle] = vrep.simxGetObjectHandle(id,'./VelodyneVPL16', vrep.simx_opmode_oneshot_wait);
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

%%
% Inicialización de valores

clear dt state P_x A_x H_x R_x sv_x G_x Q_x S_x Z_x temp_x K_x

state = [0 0 -pi 0]'; %[x y pose pose']
dt = 0.01;

P_x = 1;
R_x = 1/(10^7);
sv_x = 10^3;
G_x = 0.5*dt^2;
Q_x = G_x^2*sv_x^2;

clear P_y A_y H_y R_y sv_y G_y Q_y S_y Z_y temp_y K_y


P_y = 100;
R_y = 1/(10^7);
sv_y = 10^3;
G_y = 0.5*dt^2;
Q_y = G_y^2*sv_x^2;

clear P_pose A_pose H_pose R_pose sv_pose G_pose Q_pose S_pose Z_pose temp_pose K_pose

P_pose = diag([0.01 0.01]);
A_pose = [1 2.3*dt; 0  1];
H_pose = [0, 1];
R_pose = 1/(10^7);
sv_pose = 10^3;
G_pose = [0.5*dt^2 dt]';
Q_pose = G_pose*G_pose'*sv_pose^2;
I_pose = eye(2);

%%
% Kalman en tiempo real
close all
n_mediciones = 300;
counter = 0;

noise_x = wgn(1, n_mediciones, -40);
noise_y = wgn(1, n_mediciones, -40);
noise_pose = wgn(1, n_mediciones, -40);

[err, left_Motor] = vrep.simxGetObjectHandle(id,'./leftMotor',vrep.simx_opmode_blocking);
[err, right_Motor] = vrep.simxGetObjectHandle(id,'./rightMotor',vrep.simx_opmode_blocking);

[err] = vrep.simxSetJointTargetVelocity(id,right_Motor,3,vrep.simx_opmode_blocking);
[err] = vrep.simxSetJointTargetVelocity(id,left_Motor, 1,vrep.simx_opmode_blocking);

for j = linspace(0, (5*n_mediciones)-1, 5*n_mediciones)
    if mod(j, 5) == 0
        counter = counter + 1;
        [returnCode, GPSValuex] = vrep.simxGetFloatSignal(id,'datosGPSx', vrep.simx_opmode_buffer);
        GPSValuex_ruido = GPSValuex + noise_x(counter);
        [returnCode, GPSValuey] = vrep.simxGetFloatSignal(id,'datosGPSy', vrep.simx_opmode_buffer);
        GPSValuey_ruido = GPSValuey + noise_y(counter);
        [returnCode, GyroValuez] = vrep.simxGetFloatSignal(id,'datosGyroz', vrep.simx_opmode_buffer);
        GyroValuez_ruido = GyroValuez + noise_pose(counter);
        mediciones(:, counter) = [GPSValuex; GPSValuey; GyroValuez];
        [err, angle] = vrep.simxGetObjectOrientation(id, posehandle, -1, vrep.simx_opmode_streaming);
        [err, pos] = vrep.simxGetObjectPosition(id, posehandle, -1, vrep.simx_opmode_streaming);
        ground_truth(:, counter) = [pos(1); pos(2); angle(3)];
    end

    P_x = P_x + Q_x;
    S_x = P_x + R_x;
    K_x = P_x*1/S_x;
    Z_x = GPSValuex_ruido;                  %Para usar mediciones sin ruido quitar "_ruido" del nombre de la variable.
    temp_x = Z_x - (state(1));
    state(1) = state(1) + (K_x*temp_x);
    P_x = (1 - K_x)*P_x;
    state_temp_x = state(1);

    P_y = P_y + Q_y;
    S_y = P_y + R_y;
    K_y = P_y*1/S_y;
    Z_y = GPSValuey_ruido;                  %Para usar mediciones sin ruido quitar "_ruido" del nombre de la variable.
    temp_y = Z_y - (state(2));
    state(2) = state(2) + (K_y*temp_y);
    P_y = (1 - K_y)*P_y;
    state_temp_y = state(2);

    state(3:4) = A_pose*state(3:4);
    P_pose = A_pose*P_pose*A_pose' + Q_pose;
    S_pose = H_pose*P_pose*H_pose' + R_pose;
    K_pose = (P_pose*H_pose') * pinv(S_pose);
    Z_pose = GyroValuez_ruido;              %Para usar mediciones sin ruido quitar "_ruido" del nombre de la variable.
    temp_pose = Z_pose - (H_pose*state(3:4));
    state(3:4) = state(3:4) + (K_pose*temp_pose);
    state_temp_pose = state(3:4);
    P_pose = (I_pose - (K_pose*H_pose))*P_pose;

    if mod(j, 5) == 0 && j ~= 0
        state_history_avg(1:2, counter) = (state_history(1:2, j-3) + state_history(1:2, j-2) + state_history(1:2, j-1) + state_history(1:2, j) + state(1:2))./5;
        state_history_avg(3:4, counter) = state(3:4);
    end
    state_history(:,j+1) = state;
    pause(0.01);
    if j == 1000
        [err] = vrep.simxSetJointTargetVelocity(id,left_Motor, 1,vrep.simx_opmode_blocking);
        [err] = vrep.simxSetJointTargetVelocity(id,right_Motor,1,vrep.simx_opmode_blocking);
    end
end

%%
% graficos
close all;

figure(1)
hold on;
plot(state_history_avg(4,:));
plot(mediciones(3,:));
legend("Estimación","Medición");
title("theta'");
xlabel("Muestras");
ylabel("Velocidad [rad/s]");

figure(2)
hold on;
plot(state_history_avg(1,:));
plot(ground_truth(1,:));
legend("Estimación", "Ground Truth");
title("Posicion en x");
xlabel("Muestras");
ylabel("Posicion [m]");

figure(3)
hold on;
plot(state_history_avg(2,:));
plot(ground_truth(2,:));
legend("Estimación", "Ground Truth");
title("Posicion en y");
xlabel("Muestras");
ylabel("Posicion [m]");

modulado = mod(state_history_avg(3,:), 2*pi) - pi;

figure(4)
hold on;
plot(modulado);
plot(ground_truth(3,:));
legend("Estimación", "Ground Truth");
title("theta");
xlabel("Muestras");
ylabel("Posicion [rad]");