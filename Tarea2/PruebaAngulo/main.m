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

angle_robot = angle(3)*180/pi;

%%
% Metodo 1 para angulo sin ruido
close all; clc; clear x_delantero y_delantero z_delantero dist x_delantero_punta y_delantero_punta z_delantero_punta x_trasero y_trasero z_trasero dist x_trasero_punta y_trasero_punta z_trasero_punta 

[err, angle] = vrep.simxGetObjectOrientation(id, posehandle, -1, vrep.simx_opmode_buffer);
angle_robot = angle(3)*180/pi;

full_vect = zeros(1,1);
[returnCode, SignalValue] = vrep.simxReadStringStream(id,'datos', vrep.simx_opmode_streaming);
for n = linspace(1, 4, 4)
    [returnCode, SignalValue] = vrep.simxReadStringStream(id,'datos', vrep.simx_opmode_buffer);
    RealValue = vrep.simxUnpackFloats(SignalValue);
    full_vect = cat(2, full_vect, RealValue);
    pause(0.1)
end
full_vect = full_vect(2:end);
M = length(full_vect) / 3;
B = reshape(full_vect, [3, M]);
x = B(1, :); y = B(2, :); z = B(3, :);

%Baliza delantera
[err, pos] = vrep.simxGetObjectPosition(id, posehandle, -1, vrep.simx_opmode_buffer);

counter_delantero = 1;
counter_delantero_punta = 1;
for j = linspace(1, length(x), length(x))
    dist = sqrt((x(j)-pos(1))^2 + (y(j)-pos(2))^2);
    if z(j) >= 0.55 && z(j) <= 0.65
        if dist >= 0.6 && dist < 1.39
            x_delantero(counter_delantero) = B(1, j); y_delantero(counter_delantero) = B(2, j); z_delantero(counter_delantero) = B(3, j);
            counter_delantero = counter_delantero + 1;
        elseif dist < 1.4
            x_delantero_punta(counter_delantero_punta) = B(1, j); y_delantero_punta(counter_delantero_punta) = B(2, j); z_delantero_punta(counter_delantero_punta) = B(3, j);
            counter_delantero_punta = counter_delantero_punta +1;
        end
    end
end


m_delantero = polyfit(x_delantero, y_delantero, 1);
val = polyval(m_delantero, x_delantero);
med_x_delantero = median(x_delantero);
med_y_delantero = median(y_delantero);
med_x_punta_delantero = median(x_delantero_punta);
punta_diff_delantero = med_x_punta_delantero - med_x_delantero;

m_delantero = m_delantero(1);
m_p_delantero = -1/m_delantero;

if punta_diff_delantero >= 0
    theta_delantero = atan(m_p_delantero);
else
    theta_delantero = pi + atan(m_p_delantero);
end
angle_delantero = theta_delantero*180/pi;


%Baliza trasera
counter_trasero = 1;
counter_trasero_punta = 1;
for j = linspace(1, length(x), length(x))
    dist = sqrt((x(j)-pos(1))^2 + (y(j)-pos(2))^2);
    if z(j) > 0.7
        if dist >= 1.5
            x_trasero(counter_trasero) = B(1, j); y_trasero(counter_trasero) = B(2, j); z_trasero(counter_trasero) = B(3, j);
            counter_trasero = counter_trasero + 1;
        else
           x_trasero_punta(counter_trasero_punta) = B(1, j); y_trasero_punta(counter_trasero_punta) = B(2, j); z_trasero_punta(counter_trasero_punta) = B(3, j);
            counter_trasero_punta = counter_trasero_punta + 1;
        end
    end
end

m_trasero = polyfit(x_trasero, y_trasero, 1);
val_2 = polyval(m_trasero, x_trasero);
med_x_trasero = median(x_trasero);
med_y_trasero = median(y_trasero);
med_x_punta_trasero = median(x_trasero_punta);
punta_diff_trasero = med_x_punta_trasero - med_x_trasero;

m_trasero = m_trasero(1);
m_p_trasero = -1/m_trasero;

if punta_diff_trasero >= 0
    theta_trasero = atan(m_p_trasero);
else
    theta_trasero = pi + atan(m_p_trasero);
end
angle_trasero = theta_trasero*180/pi;


figure; %plot3(x, y, z, '.', "LineWidth", 0.5);
plot3(x_trasero, y_trasero, z_trasero, '.', "LineWidth", 0.5, "Color", [1 0 0]); hold on;
plot3(x_trasero, val_2, z_trasero, '.', "LineWidth", 0.8, "Color", [0 0 1]);
plot3(x_trasero_punta, y_trasero_punta, z_trasero_punta, '.', "LineWidth", 0.5, "Color", [0 1 0]);
plot3(med_x_trasero, med_y_trasero, z_trasero_punta, '.', "LineWidth", 0.5, "Color", [0 0 0]);
xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D Plot');
grid on;

figure; %plot3(x, y, z, '.', "LineWidth", 0.5);
plot3(x_delantero, y_delantero, z_delantero, '.', "LineWidth", 0.5, "Color", [1 0 0]); hold on;
plot3(x_delantero, val, z_delantero, '.', "LineWidth", 0.8, "Color", [0 0 1]);
plot3(x_delantero_punta, y_delantero_punta, z_delantero_punta, '.', "LineWidth", 0.5, "Color", [0 1 0]);
plot3(med_x_delantero, med_y_delantero, z_delantero_punta, '.', "LineWidth", 0.5, "Color", [0 0 0]);
xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D Plot');
grid on;

%%
% Metodo 1 para angulo con ruido
close all; clc; clear x_delantero y_delantero z_delantero dist x_delantero_punta y_delantero_punta z_delantero_punta x_trasero y_trasero z_trasero dist x_trasero_punta y_trasero_punta z_trasero_punta 

[err, angle] = vrep.simxGetObjectOrientation(id, posehandle, -1, vrep.simx_opmode_buffer);
angle_robot = angle(3)*180/pi;

full_vect = zeros(1,1);
[returnCode, SignalValue] = vrep.simxReadStringStream(id,'datos', vrep.simx_opmode_streaming);
for n = linspace(1, 4, 4)
    [returnCode, SignalValue] = vrep.simxReadStringStream(id,'datos', vrep.simx_opmode_buffer);
    RealValue = vrep.simxUnpackFloats(SignalValue);
    full_vect = cat(2, full_vect, RealValue);
    pause(0.1)
end
full_vect = full_vect(2:end);
M = length(full_vect) / 3;
B = reshape(full_vect, [3, M]);
x = B(1, :); y = B(2, :); z = B(3, :);

%Baliza delantera
[err, pos] = vrep.simxGetObjectPosition(id, posehandle, -1, vrep.simx_opmode_buffer);

counter_delantero = 1;
counter_delantero_punta = 1;
for j = linspace(1, length(x), length(x))
    dist = sqrt((x(j)-pos(1))^2 + (y(j)-pos(2))^2);
    if z(j) >= 0.55 && z(j) <= 0.7
        if dist >= 0.6 && dist < 1.39
            x_delantero(counter_delantero) = B(1, j); y_delantero(counter_delantero) = B(2, j); z_delantero(counter_delantero) = B(3, j);
            counter_delantero = counter_delantero + 1;
        elseif dist < 1.4
            x_delantero_punta(counter_delantero_punta) = B(1, j); y_delantero_punta(counter_delantero_punta) = B(2, j); z_delantero_punta(counter_delantero_punta) = B(3, j);
            counter_delantero_punta = counter_delantero_punta +1;
        end
    end
end

power = -40;
noise_delantero = wgn(1, length(x_delantero), power);
noise_punta_delantero = wgn(1, length(x_delantero_punta), power);

x_delantero = x_delantero + noise_delantero;
y_delantero = y_delantero + noise_delantero;

x_delantero_punta = x_delantero_punta + noise_punta_delantero;
y_delantero_punta = y_delantero_punta + noise_punta_delantero;


m_delantero = polyfit(x_delantero, y_delantero, 1);
val = polyval(m_delantero, x_delantero);
med_x_delantero = median(x_delantero);
med_y_delantero = median(y_delantero);
med_x_punta_delantero = median(x_delantero_punta);
punta_diff_delantero = med_x_punta_delantero - med_x_delantero;

m_delantero = m_delantero(1);
m_p_delantero = -1/m_delantero;

if punta_diff_delantero >= 0
    theta_delantero = atan(m_p_delantero);
else
    theta_delantero = pi + atan(m_p_delantero);
end
angle_delantero = theta_delantero*180/pi;


%Baliza trasera
counter_trasero = 1;
counter_trasero_punta = 1;
for j = linspace(1, length(x), length(x))
    dist = sqrt((x(j)-pos(1))^2 + (y(j)-pos(2))^2);
    if z(j) > 0.7
        if dist >= 1.5
            x_trasero(counter_trasero) = B(1, j); y_trasero(counter_trasero) = B(2, j); z_trasero(counter_trasero) = B(3, j);
            counter_trasero = counter_trasero + 1;
        else
           x_trasero_punta(counter_trasero_punta) = B(1, j); y_trasero_punta(counter_trasero_punta) = B(2, j); z_trasero_punta(counter_trasero_punta) = B(3, j);
            counter_trasero_punta = counter_trasero_punta + 1;
        end
    end
end

power = -40;
noise_trasero = wgn(1, length(x_trasero), power);
noise_punta_trasero = wgn(1, length(x_trasero_punta), power);

x_trasero = x_trasero + noise_trasero;
y_trasero = y_trasero + noise_trasero;

x_trasero_punta = x_trasero_punta + noise_punta_trasero;
y_trasero_punta = y_trasero_punta + noise_punta_trasero;

m_trasero = polyfit(x_trasero, y_trasero, 1);
val_2 = polyval(m_trasero, x_trasero);
med_x_trasero = median(x_trasero);
med_y_trasero = median(y_trasero);
med_x_punta_trasero = median(x_trasero_punta);
punta_diff_trasero = med_x_punta_trasero - med_x_trasero;

m_trasero = m_trasero(1);
m_p_trasero = -1/m_trasero;

if punta_diff_trasero >= 0
    theta_trasero = atan(m_p_trasero);
else
    theta_trasero = pi + atan(m_p_trasero);
end
angle_trasero = theta_trasero*180/pi;


figure; %plot3(x, y, z, '.', "LineWidth", 0.5);
plot3(x_trasero, y_trasero, z_trasero, '.', "LineWidth", 0.5, "Color", [1 0 0]); hold on;
plot3(x_trasero, val_2, z_trasero, '.', "LineWidth", 0.8, "Color", [0 0 1]);
plot3(x_trasero_punta, y_trasero_punta, z_trasero_punta, '.', "LineWidth", 0.5, "Color", [0 1 0]);
plot3(med_x_trasero, med_y_trasero, z_trasero_punta, '.', "LineWidth", 0.5, "Color", [0 0 0]);
xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D Plot');
grid on;

figure; %plot3(x, y, z, '.', "LineWidth", 0.5);
plot3(x_delantero, y_delantero, z_delantero, '.', "LineWidth", 0.5, "Color", [1 0 0]); hold on;
plot3(x_delantero, val, z_delantero, '.', "LineWidth", 0.8, "Color", [0 0 1]);
plot3(x_delantero_punta, y_delantero_punta, z_delantero_punta, '.', "LineWidth", 0.5, "Color", [0 1 0]);
plot3(med_x_delantero, med_y_delantero, z_delantero_punta, '.', "LineWidth", 0.5, "Color", [0 0 0]);
xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D Plot');
grid on;



%%
% Metodo 2 para angulo sin ruido
close all; clc; clear x_delantero y_delantero z_delantero dist x_delantero_punta y_delantero_punta z_delantero_punta x_trasero y_trasero z_trasero dist x_trasero_punta y_trasero_punta z_trasero_punta 

[err, angle] = vrep.simxGetObjectOrientation(id, posehandle, -1, vrep.simx_opmode_buffer);
angle_robot = angle(3)*180/pi;

full_vect = zeros(1,1);
[returnCode, SignalValue] = vrep.simxReadStringStream(id,'datos', vrep.simx_opmode_streaming);
for n = linspace(1, 4, 4)
    [returnCode, SignalValue] = vrep.simxReadStringStream(id,'datos', vrep.simx_opmode_buffer);
    RealValue = vrep.simxUnpackFloats(SignalValue);
    full_vect = cat(2, full_vect, RealValue);
    pause(0.1)
end
full_vect = full_vect(2:end);
M = length(full_vect) / 3;
B = reshape(full_vect, [3, M]);
x = B(1, :); y = B(2, :); z = B(3, :);

%Baliza delantera
[err, pos] = vrep.simxGetObjectPosition(id, posehandle, -1, vrep.simx_opmode_buffer);

counter_delantero = 1;
counter_delantero_punta = 1;
for j = linspace(1, length(x), length(x))
    dist = sqrt((x(j)-pos(1))^2 + (y(j)-pos(2))^2);
    if z(j) >= 0.55 && z(j) <= 0.7
        if dist >= 0.6 && dist < 1.39
            x_delantero(counter_delantero) = B(1, j); y_delantero(counter_delantero) = B(2, j); z_delantero(counter_delantero) = B(3, j);
            counter_delantero = counter_delantero + 1;
        elseif dist < 1.4
            x_delantero_punta(counter_delantero_punta) = B(1, j); y_delantero_punta(counter_delantero_punta) = B(2, j); z_delantero_punta(counter_delantero_punta) = B(3, j);
            counter_delantero_punta = counter_delantero_punta +1;
        end
    end
end


m_delantero = polyfit(x_delantero, y_delantero, 1);
val = polyval(m_delantero, x_delantero);
med_x_delantero = mean(x_delantero);
med_y_delantero = mean(val);
med_x_punta_delantero = mean(x_delantero_punta);
med_y_punta_delantero = mean(y_delantero_punta);
punta_diff_delantero = med_x_punta_delantero - med_x_delantero;

m_p_delantero = (med_y_punta_delantero - med_y_delantero) / (med_x_punta_delantero - med_x_delantero);

%m_delantero = m_delantero(1);
%m_p_delantero = -1/m_delantero;

if punta_diff_delantero >= 0
    theta_delantero = atan(m_p_delantero);
else
    theta_delantero = pi + atan(m_p_delantero);
end
angle_delantero = theta_delantero*180/pi;


%Baliza trasera
counter_trasero = 1;
counter_trasero_punta = 1;
for j = linspace(1, length(x), length(x))
    dist = sqrt((x(j)-pos(1))^2 + (y(j)-pos(2))^2);
    if z(j) > 0.7
        if dist >= 1.4
            x_trasero(counter_trasero) = B(1, j); y_trasero(counter_trasero) = B(2, j); z_trasero(counter_trasero) = B(3, j);
            counter_trasero = counter_trasero + 1;
        else
           x_trasero_punta(counter_trasero_punta) = B(1, j); y_trasero_punta(counter_trasero_punta) = B(2, j); z_trasero_punta(counter_trasero_punta) = B(3, j);
            counter_trasero_punta = counter_trasero_punta + 1;
        end
    end
end

m_trasero = polyfit(x_trasero, y_trasero, 1);
val_2 = polyval(m_trasero, x_trasero);
med_x_trasero = mean(x_trasero);
med_y_trasero = mean(val_2);
med_x_punta_trasero = mean(x_trasero_punta);
med_y_punta_trasero = mean(y_trasero_punta);
punta_diff_trasero = med_x_punta_trasero - med_x_trasero;

m_p_trasero = (med_y_punta_trasero - med_y_trasero) / (med_x_punta_trasero - med_x_trasero);

if punta_diff_trasero >= 0
    theta_trasero = atan(m_p_trasero);
else
    theta_trasero = pi + atan(m_p_trasero);
end
angle_trasero = theta_trasero*180/pi;


figure; %plot3(x, y, z, '.', "LineWidth", 0.5);
plot3(x_trasero, y_trasero, z_trasero, '.', "LineWidth", 0.5, "Color", [1 0 0]); hold on;
plot3(x_trasero, val_2, z_trasero, '.', "LineWidth", 0.8, "Color", [0 0 1]);
plot3(x_trasero_punta, y_trasero_punta, z_trasero_punta, '.', "LineWidth", 0.5, "Color", [0 1 0]);
plot3(med_x_trasero, med_y_trasero, z_trasero_punta, '.', "LineWidth", 0.5, "Color", [0 0 0]);
xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D Plot');
grid on;

figure; %plot3(x, y, z, '.', "LineWidth", 0.5);
plot3(x_delantero, y_delantero, z_delantero, '.', "LineWidth", 0.5, "Color", [1 0 0]); hold on;
plot3(x_delantero, val, z_delantero, '.', "LineWidth", 0.8, "Color", [0 0 1]);
plot3(x_delantero_punta, y_delantero_punta, z_delantero_punta, '.', "LineWidth", 0.5, "Color", [0 1 0]);
plot3(med_x_delantero, med_y_delantero, z_delantero_punta, '.', "LineWidth", 0.5, "Color", [0 0 0]);
plot3(med_x_punta_delantero, med_y_punta_delantero, z_delantero_punta, '.', "LineWidth", 0.5, "Color", [0 0 0]);
xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D Plot');
grid on;

%%
% Metodo 2 para angulo con ruido
close all; clc; clear x_delantero y_delantero z_delantero dist x_delantero_punta y_delantero_punta z_delantero_punta x_trasero y_trasero z_trasero dist x_trasero_punta y_trasero_punta z_trasero_punta 

[err, angle] = vrep.simxGetObjectOrientation(id, posehandle, -1, vrep.simx_opmode_buffer);
angle_robot = angle(3)*180/pi;

full_vect = zeros(1,1);
[returnCode, SignalValue] = vrep.simxReadStringStream(id,'datos', vrep.simx_opmode_streaming);
for n = linspace(1, 4, 4)
    [returnCode, SignalValue] = vrep.simxReadStringStream(id,'datos', vrep.simx_opmode_buffer);
    RealValue = vrep.simxUnpackFloats(SignalValue);
    full_vect = cat(2, full_vect, RealValue);
    pause(0.1)
end
full_vect = full_vect(2:end);
M = length(full_vect) / 3;
B = reshape(full_vect, [3, M]);
x = B(1, :); y = B(2, :); z = B(3, :);

%Baliza delantera
[err, pos] = vrep.simxGetObjectPosition(id, posehandle, -1, vrep.simx_opmode_buffer);

counter_delantero = 1;
counter_delantero_punta = 1;
for j = linspace(1, length(x), length(x))
    dist = sqrt((x(j)-pos(1))^2 + (y(j)-pos(2))^2);
    if z(j) >= 0.55 && z(j) <= 0.7
        if dist >= 0.6 && dist < 1.39
            x_delantero(counter_delantero) = B(1, j); y_delantero(counter_delantero) = B(2, j); z_delantero(counter_delantero) = B(3, j);
            counter_delantero = counter_delantero + 1;
        elseif dist < 1.4
            x_delantero_punta(counter_delantero_punta) = B(1, j); y_delantero_punta(counter_delantero_punta) = B(2, j); z_delantero_punta(counter_delantero_punta) = B(3, j);
            counter_delantero_punta = counter_delantero_punta +1;
        end
    end
end

power = -40;
noise_delantero = wgn(1, length(x_delantero), power);
noise_punta_delantero = wgn(1, length(x_delantero_punta), power);

x_delantero = x_delantero + noise_delantero;
y_delantero = y_delantero + noise_delantero;

x_delantero_punta = x_delantero_punta + noise_punta_delantero;
y_delantero_punta = y_delantero_punta + noise_punta_delantero;


m_delantero = polyfit(x_delantero, y_delantero, 1);
val = polyval(m_delantero, x_delantero);
med_x_delantero = mean(x_delantero);
med_y_delantero = mean(y_delantero);
med_x_punta_delantero = mean(x_delantero_punta);
med_y_punta_delantero = mean(y_delantero_punta);
punta_diff_delantero = med_x_punta_delantero - med_x_delantero;

m_p_delantero = (med_y_punta_delantero - med_y_delantero) / (med_x_punta_delantero - med_x_delantero);

%m_delantero = m_delantero(1);
%m_p_delantero = -1/m_delantero;

if punta_diff_delantero >= 0
    theta_delantero = atan(m_p_delantero);
else
    theta_delantero = pi + atan(m_p_delantero);
end
angle_delantero = theta_delantero*180/pi;


%Baliza trasera
counter_trasero = 1;
counter_trasero_punta = 1;
for j = linspace(1, length(x), length(x))
    dist = sqrt((x(j)-pos(1))^2 + (y(j)-pos(2))^2);
    if z(j) > 0.7
        if dist >= 1.4
            x_trasero(counter_trasero) = B(1, j); y_trasero(counter_trasero) = B(2, j); z_trasero(counter_trasero) = B(3, j);
            counter_trasero = counter_trasero + 1;
        else
           x_trasero_punta(counter_trasero_punta) = B(1, j); y_trasero_punta(counter_trasero_punta) = B(2, j); z_trasero_punta(counter_trasero_punta) = B(3, j);
            counter_trasero_punta = counter_trasero_punta + 1;
        end
    end
end

power = -40;
noise_trasero = wgn(1, length(x_trasero), power);
noise_punta_trasero = wgn(1, length(x_trasero_punta), power);

x_trasero = x_trasero + noise_trasero;
y_trasero = y_trasero + noise_trasero;

x_trasero_punta = x_trasero_punta + noise_punta_trasero;
y_trasero_punta = y_trasero_punta + noise_punta_trasero;

m_trasero = polyfit(x_trasero, y_trasero, 1);
val_2 = polyval(m_trasero, x_trasero);
med_x_trasero = mean(x_trasero);
med_y_trasero = mean(y_trasero);
med_x_punta_trasero = mean(x_trasero_punta);
med_y_punta_trasero = mean(y_trasero_punta);
punta_diff_trasero = med_x_punta_trasero - med_x_trasero;

m_p_trasero = (med_y_punta_trasero - med_y_trasero) / (med_x_punta_trasero - med_x_trasero);

if punta_diff_trasero >= 0
    theta_trasero = atan(m_p_trasero);
else
    theta_trasero = pi + atan(m_p_trasero);
end
angle_trasero = theta_trasero*180/pi;


figure; %plot3(x, y, z, '.', "LineWidth", 0.5);
plot3(x_trasero, y_trasero, z_trasero, '.', "LineWidth", 0.5, "Color", [1 0 0]); hold on;
plot3(x_trasero, val_2, z_trasero, '.', "LineWidth", 0.8, "Color", [0 0 1]);
plot3(x_trasero_punta, y_trasero_punta, z_trasero_punta, '.', "LineWidth", 0.5, "Color", [0 1 0]);
plot3(med_x_trasero, med_y_trasero, z_trasero_punta, '.', "LineWidth", 0.5, "Color", [0 0 0]);
xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D Plot');
grid on;

figure; %plot3(x, y, z, '.', "LineWidth", 0.5);
plot3(x_delantero, y_delantero, z_delantero, '.', "LineWidth", 0.5, "Color", [1 0 0]); hold on;
plot3(x_delantero, val, z_delantero, '.', "LineWidth", 0.8, "Color", [0 0 1]);
plot3(x_delantero_punta, y_delantero_punta, z_delantero_punta, '.', "LineWidth", 0.5, "Color", [0 1 0]);
plot3(med_x_delantero, med_y_delantero, z_delantero_punta, '.', "LineWidth", 0.5, "Color", [0 0 0]);
plot3(med_x_punta_delantero, med_y_punta_delantero, z_delantero_punta, '.', "LineWidth", 0.5, "Color", [0 0 0]);
xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D Plot');
grid on;