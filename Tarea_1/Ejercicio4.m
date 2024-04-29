clc; clear;
close all;

%Parametros de simulacion
N = 1800;
max_F = 100;
max_tau = max_F*0.1;
step = max_tau/N;
T_k = 0.001;                %tiempo de sampling
t = linspace(0, N*T_k, N);

tau_r = zeros(N, 1);

tau_r(1:300) = max_tau*ones(300, 1);
tau_r(301:900) = -max_tau*ones(600, 1);
tau_r(901:1200) = max_tau*ones(300, 1);
tau_r(1501:1800) = max_tau*ones(300, 1);

tau_l = zeros(N, 1);

tau_l(1:300) = max_tau*ones(300, 1);
tau_l(301:900) = -max_tau*ones(600, 1);

% figure(1)
% hold on;
% plot(t, tau_l);
% plot(t, tau_r);
% legend("$\tau_l$", "$\tau_r$", "Interpreter", "latex");
% xlabel("Muestras");
% ylabel("Torque [Nm]");


%Parametros del robot
mu_e = 1;
mu_d = 0.6;
d = 0.25;
c = 0.1;
m = 30;
g = 9.8;

R = sqrt(2)*d;
J = (m*R^2)/(2);
F_g = m*g;

%Definición de vectores con condiciones iniciales

v_r = zeros(N, 1);
w_r = zeros(N, 1);

X_I = zeros(N, 1);
Y_I = zeros(N, 1);
theta_I = zeros(N, 1);

F_r = tau_r/c;
F_l = tau_l/c;

%Funcion auxiliar por partes para el efecto del roce
syms f_aux;
f_auxf = piecewise(abs(f_aux) <= 0.1*m*g*mu_e, -f_aux, abs(f_aux) > 0.1*m*g*mu_e, sign(f_aux)*m*g*mu_d);
F_rf = subs(f_auxf, f_aux, F_r);
F_lf = subs(f_auxf, f_aux, F_l);
F_rf = double(F_rf);
F_lf = double(F_lf);

F_t = F_r+F_l+F_rf+F_lf;
F_t2 = F_r-F_l+F_rf-F_lf;
F_t = double(F_t);
F_t2 = double(F_t2);

for k = linspace(1, N-1, N-1)
    v_r(k+1) = v_r(k) + T_k/m*(F_t(k));
    w_r(k+1) = w_r(k) + T_k/(J*d)*(F_t2(k));

    X_I(k+1) = X_I(k) + T_k*(cos(theta_I(k))*v_r(k));
    Y_I(k+1) = Y_I(k) + T_k*(sin(theta_I(k))*v_r(k));
    theta_I(k+1) = theta_I(k) + T_k*w_r(k);
end

modo = "Hacia adelante, ";
pause(1);
for j = linspace(1, N-1, N-1)
    if j==600
        modo = "Hacia atras, ";
    end
    if j==900
        modo = "1 sola rueda, ";
    end
    if j==1500
        modo = "Ruedas contra puestas, ";
    end
    figure(2)
    quiver(X_I(j), Y_I(j), 0.1*cos(theta_I(j)), 0.1*sin(theta_I(j)), "-or")
    xlim([-1 3])
    ylim([-2 2])
    text(-0.9, 1.8, "Modo: " + modo + "Iteacion: " + num2str(j));
    pause(T_k)
end

