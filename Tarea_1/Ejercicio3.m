%Definicion de ruta (Caso ruta circular)
clc; clear;
close all;

step = 0.01;
N = 10/step;

angle = linspace(0, 2*pi, N);
r = 5;
x = r*cos(angle);
y = r*sin(angle);
theta_r = angle + pi/2;


%Parametros del robot
T_k = 0.001;                %tiempo de sampling
d = 0.555;
t = linspace(0, N*T_k, N);

%Obtencion del perfil de velocidades
v_r = zeros(N, 1); %Vector de velocidad rueda derecha
v_l = zeros(N, 1); %Vector de velocidad rueda izquierda

for k = linspace(2, N, N-1)
    theta1 = theta_r(k);

    v_l(k-1) = cos(theta1)*(x(k)-x(k-1))/T_k + sin(theta1)*(y(k)-y(k-1))/T_k - d/2*(theta_r(k) - theta_r(k-1))/T_k;
    v_r(k-1) = cos(theta1)*(x(k)-x(k-1))/T_k + sin(theta1)*(y(k)-y(k-1))/T_k + d/2*(theta_r(k) - theta_r(k-1))/T_k;
end

figure(1)
hold on;
plot(t, [v_l v_r]);
legend("$v_{l}$", "$v_{r}$",'Interpreter','latex');
title("Perfiles de velocidad lineal de las dos ruedas")
xlabel("tiempo [s]");
ylabel("velocidad [m/s]")


%Estimación de posicion del robot a partir de los perfiles de velocidad obtenidos

x_p_r = zeros(N, 1);
x_p_r(1) = 5;
y_p_r = zeros(N, 1);
theta_p_r = zeros(N, 1);
theta_p_r(1) = theta_r(1);     %Pose inicial del robot

for k = linspace(1, N-1, N-1)
    theta1 = theta_p_r(k);

    x_p_r(k+1) = x_p_r(k) + T_k/2*cos(theta1)*(v_r(k)+v_l(k));
    y_p_r(k+1) = y_p_r(k) + T_k/2*sin(theta1)*(v_r(k)+v_l(k));
    theta_p_r(k+1) = theta_p_r(k) + T_k/d*(v_r(k)-v_l(k));
end

%Estimación de posicion del trailer a partir de los perfiles de velocidad obtenidos

x_p_t = zeros(N, 1);
x_p_t(1) = x_p_r(1) - 2*cos(theta_p_r(1));
y_p_t = zeros(N, 1);
y_p_t(1) = y_p_r(1) - 2*sin(theta_p_r(1));
theta_p_t = zeros(N, 1);
theta_p_t(1) = theta_r(1);     %Pose inicial del trailer

for k = linspace(1, N-1, N-1)
    theta1 = theta_p_t(k);
    theta2 = theta_p_r(k);
    beta = theta2 - theta1;

    x_p_t(k+1) = x_p_t(k) + T_k*cos(theta1)*(sin(beta)*(v_r(k)-v_l(k))/d + 1/2*cos(beta)*(v_r(k)+v_l(k)));
    y_p_t(k+1) = y_p_t(k) + T_k*sin(theta1)*(sin(beta)*(v_r(k)-v_l(k))/d + 1/2*cos(beta)*(v_r(k)+v_l(k)));
    theta_p_t(k+1) = theta_p_t(k) + T_k*(-cos(beta)*(v_r(k)-v_l(k))/d + 1/2*sin(beta)*(v_r(k)+v_l(k)));
end

pause(1);
for j = linspace(1, length(x)-1, length(x)-1)
    figure(2)
    quiver(x_p_r(j), y_p_r(j), 0.6*0.5*cos(theta_p_r(j)), 0.5*sin(theta_p_r(j)), "-or")
    text(x_p_r(j), y_p_r(j)+1, "Robot");
    hold on;
    quiver(x_p_t(j), y_p_t(j), 0.6*0.5*cos(theta_p_t(j)), 0.5*sin(theta_p_t(j)), "-or")
    text(x_p_t(j), y_p_t(j)+1, "Trailer");
    xlim([-10 10])
    ylim([-10 10])
    plot(x, y, 'LineStyle',':', 'Color','blue');
    plot(5, 0, 'go');
    text(5.5, 0, "Origen/Destino");
    hold off;
    pause(T_k)
end

%%
%Definicion de ruta (Ruta recta)
clc; clear;
close all;

step = 0.01;
N = 10/step;

x = linspace(0, 10, N)';
y = zeros(N, 1);
theta_r = zeros(N, 1);

%Parametros del robot
T_k = 0.001;                %tiempo de sampling
d = 0.555;
t = linspace(0, N*T_k, N);

%Obtencion del perfil de velocidades
v_r = zeros(N, 1); %Vector de velocidad rueda derecha
v_l = zeros(N, 1); %Vector de velocidad rueda izquierda

for k = linspace(2, N, N-1)
    theta1 = theta_r(k);

    v_l(k-1) = cos(theta1)*(x(k)-x(k-1))/T_k + sin(theta1)*(y(k)-y(k-1))/T_k - d/2*(theta_r(k) - theta_r(k-1))/T_k;
    v_r(k-1) = cos(theta1)*(x(k)-x(k-1))/T_k + sin(theta1)*(y(k)-y(k-1))/T_k + d/2*(theta_r(k) - theta_r(k-1))/T_k;
end

figure(1)
hold on;
plot(t, [v_l v_r]);
legend("$v_{l}$", "$v_{r}$",'Interpreter','latex');
title("Perfiles de velocidad lineal de las dos ruedas")
xlabel("tiempo [s]");
ylabel("velocidad [m/s]")


%Estimación de posicion del robot a partir de los perfiles de velocidad obtenidos

x_p_r = zeros(N, 1);
y_p_r = zeros(N, 1);
theta_p_r = zeros(N, 1);
theta_p_r(1) = theta_r(1);     %Pose inicial del robot

for k = linspace(1, N-1, N-1)
    theta1 = theta_p_r(k);

    x_p_r(k+1) = x_p_r(k) + T_k/2*cos(theta1)*(v_r(k)+v_l(k));
    y_p_r(k+1) = y_p_r(k) + T_k/2*sin(theta1)*(v_r(k)+v_l(k));
    theta_p_r(k+1) = theta_p_r(k) + T_k/d*(v_r(k)-v_l(k));
end

%Estimación de posicion del trailer a partir de los perfiles de velocidad obtenidos

x_p_t = zeros(N, 1);
x_p_t(1) = x_p_r(1) - 2*cos(theta_p_r(1));
y_p_t = zeros(N, 1);
y_p_t(1) = y_p_r(1) - 2*sin(theta_p_r(1));
theta_p_t = zeros(N, 1);
theta_p_t(1) = theta_r(1);     %Pose inicial del trailer

for k = linspace(1, N-1, N-1)
    theta1 = theta_p_t(k);
    theta2 = theta_p_r(k);
    beta = theta2 - theta1;

    x_p_t(k+1) = x_p_t(k) + T_k*cos(theta1)*(sin(beta)*(v_r(k)-v_l(k))/d + 1/2*cos(beta)*(v_r(k)+v_l(k)));
    y_p_t(k+1) = y_p_t(k) + T_k*sin(theta1)*(1.8*sin(beta)*(v_r(k)-v_l(k))/d + 1/2*cos(beta)*(v_r(k)+v_l(k)));
    theta_p_t(k+1) = theta_p_t(k) + T_k*(-cos(beta)*(v_r(k)-v_l(k))/d + 1/2*sin(beta)*(v_r(k)+v_l(k)));
end

for j = linspace(1, length(x)-1, length(x)-1)
    figure(2)
    quiver(x_p_r(j), y_p_r(j), 0.6*0.5*cos(theta_p_r(j)), 0.5*sin(theta_p_r(j)), "-or")
    text(x_p_r(j), y_p_r(j)-0.1, "Robot");
    hold on;
    quiver(x_p_t(j), y_p_t(j), 0.6*0.5*cos(theta_p_t(j)), 0.5*sin(theta_p_t(j)), "-or")
    text(x_p_t(j), y_p_t(j)-0.1, "Trailer");
    xlim([-2 11])
    ylim([-1 1])
    line([0 10], [0 0], 'LineStyle',':', 'Color','blue');
    plot(0, 0, 'go');
    plot(10, 0, 'go');
    text(-0.5, 0.1, "Origen");
    text(9.5, 0.1, "Destino");
    hold off;
    pause(T_k)
end