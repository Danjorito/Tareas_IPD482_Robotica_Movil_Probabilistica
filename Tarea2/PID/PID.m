% Conexión a Coppelia 

clc; clear;
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1', 19000, true, true, 5000, 5);

%% Generación del camino

N_camino = 300;

% Sección recta

n_recta = N_camino/2; % Nro de puntos del camino
y_recta = linspace(-2,0,n_recta);
x_recta = -2*ones(1,n_recta);

% Semi-circunferencia

n_circ = N_camino/2;
radius = 2;
x_circ = linspace(-2,2,n_circ);
y_circ = sqrt(radius.^2 - x_circ.^2);

% Union de ambos

x_camino = [x_recta, x_circ];
y_camino = [y_recta, y_circ];

%figure;
%scatter(x_camino,y_camino);


%%


% Get Handles

[lm_err, left_Motor] = vrep.simxGetObjectHandle(id,'./leftMotor',vrep.simx_opmode_blocking);
[rm_err, right_Motor] = vrep.simxGetObjectHandle(id,'./rightMotor',vrep.simx_opmode_blocking);

[rp_err, robot_pose] = vrep.simxGetObjectHandle(id,'./robot_pose',vrep.simx_opmode_blocking);




%%

% Parameters

% Kp = 0.1;
% Kd = 0.8;
% Ki = 0.1;

vel = 0.2;
Kp = 0.1;
Kd = 1;
Ki = 0.06;
steer = 0;
err_anterior=0;
epsilon = 0.8;
k = 0.1;
while 1
    
    [p_err, position] = vrep.simxGetObjectPosition(id, robot_pose,-1,vrep.simx_opmode_streaming);
    [err, orientation] = vrep.simxGetObjectOrientation(id, robot_pose,-1,vrep.simx_opmode_streaming);

    angle = orientation(3)*180/pi;
    x_real = position(1);
    y_real = position(2);
    
    min_dist = inf;
    x_min = 0;
    y_min = 0;
    % Calculo del punto mas cercano al robot
    for i = 1:length(x_camino)-1
        dist_temp = sqrt((x_real - x_camino(i))^2 + (y_real - y_camino(i))^2);
        if dist_temp < min_dist
            min_dist = dist_temp;
            x_min = x_camino(i);
            y_min = y_camino(i);
        end
    end
    
    stop = sqrt( (x_real-x_camino(N_camino))^2 + (y_real-y_camino(N_camino))^2);
    if stop< epsilon
        [lm_err] = vrep.simxSetJointTargetVelocity(id,0,izq_vel,vrep.simx_opmode_blocking);
        [rm_err] = vrep.simxSetJointTargetVelocity(id,0,der_vel,vrep.simx_opmode_blocking);
        break;
    end
    
    x_diff = x_real - x_min;
    y_diff = y_real - y_min;
    err = sqrt((x_diff)^2 + (y_diff)^2);
    
    pid = Kp*err + Kd*(err-err_anterior)+ Ki*(err+err_anterior);
    if angle >= -30
        if abs(x_diff) < abs(y_diff)
            if y_diff <= 0
                steer = pid;
            else
                steer = -pid;
            end
        else
            if x_diff <= 0
                steer = -pid;
            else
                steer = pid;
            end 
        end
    else
        if abs(x_diff) < abs(y_diff)
            if y_diff <= 0
                steer = -pid - k;
            else
                steer = pid;
            end
        else
            if x_diff <= 0
                steer = pid;
            else
                steer = -pid-k;
            end 
        end
    end


    err_anterior = err;
    izq_vel = vel - steer;
    der_vel = vel + steer;
    
    [lm_err] = vrep.simxSetJointTargetVelocity(id,left_Motor,izq_vel,vrep.simx_opmode_blocking);
    [rm_err] = vrep.simxSetJointTargetVelocity(id,right_Motor,der_vel,vrep.simx_opmode_blocking);
    %pause(time);
end




%%
% Parameters
%delta_t = 0.1;
% vel = 0.1;
% 
% Kp = 0.2;
% Kd = 0*0.01;
% Ki = 0*0.4;
% 
% steer = 0;
% err_anterior = 0;
% err_acum = 0;
% 
% for i = 1:length(x_camino)-1
%     x_sgte = x_camino(i+1);
%     y_sgte = y_camino(i+1);
% 
%     x_diff = x_sgte - x_camino(i);
%     y_diff = y_sgte - y_camino(i);
% 
%     dist = sqrt(y_diff^2 + x_diff.^2);
%     time = dist/vel; 
% 
%     izq_vel = vel - steer;
%     der_vel = vel + steer;
% 
%     [lm_err] = vrep.simxSetJointTargetVelocity(id,left_Motor,izq_vel,vrep.simx_opmode_blocking);
%     [rm_err] = vrep.simxSetJointTargetVelocity(id,right_Motor,der_vel,vrep.simx_opmode_blocking);
%     pause(time);
% 
%     [p_err, position] = vrep.simxGetObjectPosition(id, robot_pose,-1,vrep.simx_opmode_streaming);
%     %[o_err, orientation] = vrep.simxGetObjectOrientation(id, robot_pose,-1,vrep.simx_opmode_streaming);
%     x_real = position(1);
%     y_real = position(2);
% 
%     %disp([x_real,y_real])
%     %>
% 
%     x_err = x_real - x_sgte;
%     y_err = y_real - y_sgte;
%     err = sqrt(x_err^2 + y_err^2);
% 
% 
% 
%     if x_sgte < x_real && y_sgte < y_real
%         steer = Kp*err + Kd*(err - err_anterior) + Ki*(err + err_anterior);
%     elseif x_sgte < x_real && y_sgte > y_real
%         steer = -Kp*err - Kd*(err - err_anterior) -Ki*(err + err_anterior);
%     elseif x_sgte > x_real && y_sgte < y_real
%         steer = -Kp*err - Kd*(err - err_anterior)-Ki*(err + err_anterior);
%     else
%         steer = Kp*err + Kd*(err - err_anterior)+Ki*(err + err_anterior);
%     end
% 
%     err_anterior = err;
%    %err_acum = err + err_acum;
% end
