%Definicion de ruta
clc; clear;
close all;

step = 0.01;
N = 10/step;
x = linspace(0, 10, N)';
y = zeros(N, 1);
theta_r = linspace(0, 8*pi, N)';

%Parametros del robot
T_k = 0.001;                %tiempo de sampling
R = 0.45/2;                 %radio del robot
t = linspace(0, N*T_k, N);

%Obtencion del perfil de velocidades
v = zeros(N, 3); %Vector de velocidades v1, v2 y v3

for k = linspace(2, N, N-1)
    theta1 = theta_r(k);
    
    v(k, 1) = 1/2*(2*cos(theta1)*(x(k)-x(k-1))/T_k + 2*sin(theta1)*(y(k)-y(k-1))/T_k - R*(theta_r(k)-theta_r(k-1))/T_k);
    v(k, 2) = 1/2*((sqrt(3)*sin(theta1)-cos(theta1))*(x(k)-x(k-1))/T_k - (sin(theta1)+sqrt(3)*cos(theta1))*(y(k)-y(k-1))/T_k - R*(theta_r(k)-theta_r(k-1))/T_k);
    v(k, 3) = 1/2*(-1*(sqrt(3)*sin(theta1)+cos(theta1))*(x(k)-x(k-1))/T_k + (sqrt(3)*cos(theta1)-sin(theta1))*(y(k)-y(k-1))/T_k - R*(theta_r(k)-theta_r(k-1))/T_k);
end

figure(1)
hold on;
plot(t, v);
legend("$v_{W1}$", "$v_{W2}$", "$v_{W3}$",'Interpreter','latex');
title("Perfiles de velocidad lineal de las tres ruedas")
xlabel("tiempo [s]");
ylabel("velocidad [m/s]")

x_p = zeros(length(x), 1);
y_p = zeros(length(x), 1);
theta_r_p = zeros(length(x), 1);

for k = linspace(1, length(x)-1, length(x)-1)
    theta_1 = theta_r_p(k);

    x_p(k+1) = x_p(k) + 2*T_k/3*(cos(theta_1)*v(k, 1) + sin(theta_1-pi/6)*v(k, 2) - sin(theta_1+pi/6)*v(k, 3));
    y_p(k+1) = y_p(k) + 2*T_k/3*(sin(theta_1)*v(k, 1) - cos(theta_1-pi/6)*v(k, 2) + cos(theta_1+pi/6)*v(k, 3));
    theta_r_p(k+1) = theta_r_p(k) - (2*T_k)/(3*R)*(v(k, 1) + v(k, 2) + v(k, 3));
    
end


pause(1);
for j = linspace(1, length(x)-1, length(x)-1)
    figure(2)
    quiver(x_p(j), y_p(j), 0.6*0.5*cos(theta_r_p(j)), 0.5*sin(theta_r_p(j)), "-or")
    xlim([-1 11])
    ylim([-10 10])
    hold on;
    line([0 10], [0 0], 'LineStyle',':', 'Color','blue');
    plot(0, 0, 'go');
    plot(10, 0, 'go');
    text(-0.5, 1, "Origen");
    text(9.5, 1, "Destino");

    pause(T_k)
    hold off;
end