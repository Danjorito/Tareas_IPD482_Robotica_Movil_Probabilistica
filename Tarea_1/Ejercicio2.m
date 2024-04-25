%%
%Definicion de ruta
clc; clear;
close all;

step = 0.01;
N = 10/step;
x = linspace(0, 10, N)';
%x = zeros(N, 1);
%y = linspace(0, 10, N)';
y = zeros(N, 1);
%theta_r = zeros(N, 1);
theta_r = linspace(0, 8*pi, N)';
%theta_r(1:400) = linspace(0, pi, N/2-100)';
%theta_r(401:800) = linspace(pi, 0, N/2-100)';
%theta_r(1:100) = linspace(0, 5*pi/4, 100);
%theta_r(101:1000) = 5*pi/4;

%Parametros del robot
T_k = 0.001;                %tiempo de sampling
R = 0.45/2;                %radio del robot
t = linspace(0, N*T_k, N);

%Obtencion del perfil de velocidades
v = zeros(length(x), 3); %Vector de velocidades v1, v2 y v3

for k = linspace(2, N, N-1)
    theta_1 = theta_r(k);

    v(k, 1) = -sin(theta_1)*(x(k) - x(k-1))/T_k + cos(theta_1)*(y(k) - y(k-1))/T_k + R*(theta_r(k) - theta_r(k-1))/T_k;
    v(k, 2) = -cos(theta_1+pi/6)*(x(k) - x(k-1))/T_k - sin(theta_1+pi/6)*(y(k) - y(k-1))/T_k + R*(theta_r(k) - theta_r(k-1))/T_k;
    v(k, 3) = cos(theta_1-pi/6)*(x(k) - x(k-1))/T_k + sin(theta_1-pi/6)*(y(k) - y(k-1))/T_k + R*(theta_r(k) - theta_r(k-1))/T_k;
end

figure(1)
hold on;
plot(t, v);
legend("v1", "v2", "v3");

x_p = zeros(length(x), 1);
y_p = zeros(length(x), 1);
theta_r_p = zeros(length(x), 1);

for k = linspace(1, length(x)-1, length(x)-1)
    theta_1 = theta_r_p(k);
    x_p(k+1) = x_p(k)+1/3*T_k*(-2*sin(theta_1)*v(k, 1) + (sin(theta_1)-sqrt(3)*cos(theta_1))*v(k, 2) + (sin(theta_1)+sqrt(3)*cos(theta_1))*v(k, 3));
    y_p(k+1) = y_p(k)+1/3*T_k*(2*cos(theta_1)*v(k, 1) + (-sqrt(3)*sin(theta_1)-cos(theta_1))*v(k, 2) + (sqrt(3)*sin(theta_1)-cos(theta_1))*v(k, 3));
    theta_r_p(k+1) = theta_r_p(k) + (1/3*T_k/(R))*(v(k, 1)+v(k, 2)+v(k, 3));
end

pause(1);
for j = linspace(1, length(x)-1, length(x)-1)
    figure(2)
    quiver(x_p(j), y_p(j), 0.5*cos(theta_r_p(j)), 0.5*sin(theta_r_p(j)), "-or")
    xlim([-10 10])
    ylim([-10 10])
    pause(T_k)
end