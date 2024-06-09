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
% Metodo 1 para angulo
close all; clc; clear x_delantero y_delantero z_delantero distance

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
for j = linspace(1, length(x), length(x))
    dist = sqrt((x(j)-pos(1))^2 + (y(j)-pos(2))^2);
    if z(j) >= 0.5 && z(j) <= 0.7
        if dist <= 1.2
            x_delantero(counter_delantero) = B(1, j); y_delantero(counter_delantero) = B(2, j); z_delantero(counter_delantero) = B(3, j);
            counter_delantero = counter_delantero + 1;
        end
    end
end


m_delantero = polyfit(x_delantero, y_delantero, 1);
val = polyval(m_delantero, x_delantero);
m_delantero = m_delantero(1);
m_p_delantero = -1/m_delantero;
theta_delantero = atan(m_p_delantero);
angle_delantero = theta_delantero*180/pi;

%Baliza trasera
counter_trasero = 1;
for j = linspace(1, length(x), length(x))
    if z(j) >= 0.7
        x_trasero(counter_trasero) = B(1, j); y_trasero(counter_trasero) = B(2, j); z_trasero(counter_trasero) = B(3, j);
        counter_trasero = counter_trasero + 1;
    end
end

m_trasero = polyfit(x_trasero, y_trasero, 1);
val_2 = polyval(m_trasero, x_trasero);
m_trasero = m_trasero(1);
m_p_trasero = -1/m_trasero;
theta_trasero = atan(m_p_trasero);
angle_trasero = theta_trasero*180/pi;


figure; %plot3(x, y, z, '.', "LineWidth", 0.5);
plot3(x_trasero, y_trasero, z_trasero, '.', "LineWidth", 0.5, "Color", [1 0 0]); hold on;
plot3(x_trasero, val_2, z_trasero, '.', "LineWidth", 0.8, "Color", [0 0 1]);
xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D Plot');
grid on;

figure; %plot3(x, y, z, '.', "LineWidth", 0.5);
plot3(x_delantero, y_delantero, z_delantero, '.', "LineWidth", 0.5, "Color", [1 0 0]); hold on;
plot3(x_delantero, val, z_delantero, '.', "LineWidth", 0.8, "Color", [0 0 1]);
xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D Plot');
grid on;

%%
% Metodo 2 para angulo
close all; clc; clear x_delantero y_delantero z_delantero distance

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
counter_delantero = 1;
for j = linspace(1, length(x), length(x))
    dist = sqrt((x(j)-pos(1))^2 + (y(j)-pos(2))^2);
    disp(dist);
    if z(j) >= 0.5 && z(j) <= 0.7
        if dist <= 1.2
            x_delantero(counter_delantero) = B(1, j); y_delantero(counter_delantero) = B(2, j); z_delantero(counter_delantero) = B(3, j);
            counter_delantero = counter_delantero + 1;
        end
    end
end

[err, angle] = vrep.simxGetObjectOrientation(id, posehandle, -1, vrep.simx_opmode_buffer);
[err, pos] = vrep.simxGetObjectPosition(id, posehandle, -1, vrep.simx_opmode_buffer);

m_delantero = polyfit(x_delantero, y_delantero, 1);
val = polyval(m_delantero, x_delantero);

med_x = median(x_delantero);
med_y = median(val);
avg_x = mean(x_delantero);
avg_y = mean(val);

dist_x = pos(1) - avg_x;
dist_y = pos(2) - avg_y;

dist_tractor_delantero = sqrt((dist_x)^2 + (dist_y)^2);
Lh1_delantero = 0.4;
L1_delantero = 0.5;

alpha   = acos((dist_tractor_delantero^2 + L1_delantero^2 - Lh1_delantero^2)/(2*dist_tractor_delantero*L1_delantero));
alpha_angle = alpha*180/pi;

beta_sin = asin(dist_y/dist_tractor_delantero);
beta_sin_angle = beta_sin*180/pi;
beta_cos = acos(dist_x/dist_tractor_delantero);
beta_cos_angle = beta_cos*180/pi;
beta_tan = atan(dist_y/dist_x);
beta_tan_angle = beta_tan*180/pi;


beta    = acos((dist_tractor_delantero^2 + dist_x^2 - dist_y^2)/(2*dist_tractor_delantero*dist_x));
beta_angle = beta*180/pi;

if dist_x > 0 && dist_y > 0
    theta_1 = alpha + beta_tan;
elseif dist_x < 0 && dist_y > 0
    theta_1 = pi - (alpha - beta_tan);
elseif dist_x > 0 && dist_y < 0
    theta_1 = -alpha + beta_tan;
else
    theta_1 = alpha + beta_tan - pi;
end

theta_angle = (theta_1*180/pi);


%Baliza trasera
counter_trasero = 1;
for j = linspace(1, length(x), length(x))
    if z(j) >= 0.7
        x_trasero(counter_trasero) = B(1, j); y_trasero(counter_trasero) = B(2, j); z_trasero(counter_trasero) = B(3, j);
        counter_trasero = counter_trasero + 1;
    end
end

m_trasero = polyfit(x_trasero, y_trasero, 1);
m_trasero = m_trasero(1);
m_p_trasero = -1/m_trasero;
theta_trasero = atan(m_p_trasero);
angle_trasero = theta_trasero*180/pi;


figure; %plot3(x, y, z, '.', "LineWidth", 0.5);
plot3(x_delantero, val, z_delantero, '.', "LineWidth", 0.5, "Color", [1 0 0]); hold on;
plot3(med_x, med_y, z_delantero, '.', "LineWidth", 0.8, "Color", [0 0 1]);
xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D Plot');
grid on;