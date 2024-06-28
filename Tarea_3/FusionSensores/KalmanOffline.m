% Carga de datos
clear; clc; close all
load data.mat;

noise_x = wgn(1, size(mediciones, 2), -40);
noise_y = wgn(1, size(mediciones, 2), -40);
noise_pose = wgn(1, size(mediciones, 2), -40);

mediciones_ruido = [mediciones(1,:)+noise_x; mediciones(2,:)+noise_y; mediciones(3,:)+noise_pose];

%%
% Inicialización Kalman

clear dt state P_x A_x H_x R_x sv_x G_x Q_x S_x Z_x temp_x K_x

state = [0.0 0.0 0.0 0.0]'; %[x y pose pose']
dt = 0.0136;

P_x = 1;
R_x = 1/(10^5);
sv_x = 10^3;
G_x = 0.5*dt^2;
Q_x = G_x^2*sv_x^2;

clear P_y A_y H_y R_y sv_y G_y Q_y S_y Z_y temp_y K_y

P_y = 1;
R_y = 1/(10^5);
sv_y = 10^3;
G_y = 0.5*dt^2;
Q_y = G_y^2*sv_y^2;

clear P_pose A_pose H_pose R_pose sv_pose G_pose Q_pose S_pose Z_pose temp_pose K_pose

P_pose = diag([0.01 0.01]);
A_pose = [1 dt; 0  1];
H_pose = [0, 1];
R_pose = 1/(10^5);
sv_pose = 10^3;
G_pose = [0.5*dt^2 dt]';
Q_pose = G_pose*G_pose'*sv_pose^2;
I_pose = eye(2);

%%
% Procesado offline
close all
state_history = zeros(4, 5*size(mediciones, 2));
counter = 0;
for j = linspace(0, (5*size(mediciones, 2))-1, 5*size(mediciones, 2))
    if mod(j, 5) == 0
        counter = counter + 1;
    end

    P_x = P_x + Q_x;
    S_x = P_x + R_x;
    K_x = P_x*1/S_x;
    Z_x = mediciones_ruido(1, counter);  %Para usar mediciones sin ruido quitar "_ruido" del nombre de la variable.
    temp_x = Z_x - (state(1));
    state(1) = state(1) + (K_x*temp_x);
    P_x = (1 - K_x)*P_x;
    state_temp_x = state(1);

    P_y = P_y + Q_y;
    S_y = P_y + R_y;
    K_y = P_y*1/S_y;
    Z_y = mediciones_ruido(2, counter);  %Para usar mediciones sin ruido quitar "_ruido" del nombre de la variable.
    temp_y = Z_y - (state(2));
    state(2) = state(2) + (K_y*temp_y);
    P_y = (1 - K_y)*P_y;
    state_temp_y = state(2);

    state(3:4) = A_pose*state(3:4);
    P_pose = A_pose*P_pose*A_pose' + Q_pose;
    S_pose = H_pose*P_pose*H_pose' + R_pose;
    K_pose = (P_pose*H_pose') * pinv(S_pose);
    Z_pose = mediciones_ruido(3, counter);  %Para usar mediciones sin ruido quitar "_ruido" del nombre de la variable.
    temp_pose = Z_pose - (H_pose*state(3:4));
    state(3:4) = state(3:4) + (K_pose*temp_pose);
    state_temp_pose = state(3:4);
    P_pose = (I_pose - (K_pose*H_pose))*P_pose;

    if mod(j, 5) == 0 && j ~= 0
        state_history_avg(1:2, counter) = (state_history(1:2, j-3) + state_history(1:2, j-2) + state_history(1:2, j-1) + state_history(1:2, j) + state(1:2))./5;
        state_history_avg(3:4, counter) = state(3:4);
    end
    state_history(:,j+1) = state;
end

extended_lapse = (1:5:2499);

figure(1);
hold on;
plot(extended_lapse, mediciones(1,:));
plot(state_history(1,:));
plot(extended_lapse, pose_history(1,:));
title("posicion x");
legend("Mediciones", "Kalmann", "pose");
xlabel("Muestras");
ylabel("Posición [m]");

figure(2);
hold on;
plot(extended_lapse, mediciones(2,:));
plot(state_history(2,:));
plot(extended_lapse, pose_history(2,:));
title("posicion y");
legend("Mediciones", "Kalmann", "Ground Truth");
xlabel("Muestras");
ylabel("Posición [m]");

figure(3);
hold on;
plot(extended_lapse, mediciones(3,:));
plot(state_history(4,:));
title("theta'");
legend("Mediciones", "Kalman");
xlabel("Muestras");
ylabel("Posición [rad/s]");

figure(4);
hold on;
plot(extended_lapse, pose_history(3,:));
plot(state_history(3,:));
title("theta")
legend("pose", "Kalman");
xlabel("Muestras");
ylabel("Posición [rad]");
