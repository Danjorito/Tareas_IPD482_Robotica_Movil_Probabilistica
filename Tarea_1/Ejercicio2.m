clc; clear;

%Definicion de ruta
step = 0.05;
N = 10/step;
%x = linspace(0, 10, N);
x = zeros(N, 1);
y = zeros(N, 1);
theta_r = linspace(0, 2*pi, length(x))';
%theta_r = zeros(length(x), 1);

%Parametros del robot
T_k = 0.0001;                %tiempo de sampling
R = 0.45/2;                %radio del robot

%Obtencion del perfil de velocidades
v = zeros(length(x), 3); %Vector de velocidades v1, v2 y v3

for k = linspace(1, length(x)-1, length(x)-1)
    theta_1 = theta_r(k);
    theta_2 = theta_1 + 120*pi/180;
    theta_3 = theta_2 + 120*pi/180;

    v(k+1, 1) = -sin(theta_1)*-1*(x(k) - x(k+1))/T_k + cos(theta_1)*-1*(y(k) - y(k+1))/T_k + R*-1*(theta_r(k) - theta_r(k+1))/T_k;
    v(k+1, 2) = -sin(theta_1+theta_2)*-1*(x(k) - x(k+1))/T_k + cos(theta_1+theta_2)*-1*(y(k) - y(k+1))/T_k + R*-1*(theta_r(k) - theta_r(k+1))/T_k;
    v(k+1, 3) = -sin(theta_1+theta_3)*-1*(x(k) - x(k+1))/T_k + cos(theta_1+theta_3)*-1*(y(k) - y(k+1))/T_k + R*-1*(theta_r(k) - theta_r(k+1))/T_k;
end

x_p = zeros(length(x), 1);
y_p = zeros(length(x), 1);
theta_r_p = zeros(length(x), 1);

for k = linspace(1, length(x)-1, length(x)-1)
    theta_1 = theta_r_p(k);
    theta_2 = theta_1 + 120*pi/180;
    theta_3 = theta_2 + 120*pi/180;

    x_p(k+1) = x_p(k)+2/3*T_k*(-sin(theta_1)*v(k, 1) -sin(theta_1+theta_2)*v(k, 2) -sin(theta_1+theta_3)*v(k, 3));
    y_p(k+1) = y_p(k)+2/3*T_k*(cos(theta_1)*v(k, 1) +cos(theta_1+theta_2)*v(k, 2) +cos(theta_1+theta_3)*v(k, 3));
    theta_r_p(k+1) = theta_r_p(k) + (2/3*T_k/(2*R))*(v(k, 1)+v(k, 2)+v(k, 3));
end

disp('end');

pause(1);
for j = linspace(1, length(x)-1, length(x)-1)
    quiver(x_p(j), y_p(j), 0.5*cos(theta_r_p(j)), 0.5*sin(theta_r_p(j)), "-or")
    xlim([-10 10])
    ylim([-10 10])
    pause(T_k)
end