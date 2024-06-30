clc; close all; clear all;

%***********************************************************************************************************************************************************
%***********************************************        SETUP         **************************************************************************************
%***********************************************************************************************************************************************************


% Robot
L = 0.381;

% PI control
Kp = 0.2;
Ki = 0.002;
pid = 0;
previous_err =0;
acumulated_error = 0;

% Velocity profiles 
T_k = 0.1;
radius = 4;
N_points = 100;

x_min = 0;
y_min = 0;

x_acum = zeros(1,N_points);
y_acum = zeros(1,N_points);
theta_acum = zeros(1,N_points);

x_acum_local = zeros(1,N_points);
y_acum_local = zeros(1,N_points);

x_min_acum = zeros(1,N_points);
y_min_acum = zeros(1,N_points);


% Coppelia related
% Connection
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
% Handles
vrep_id = vrep.simxStart('127.0.0.1', 19000, true, true, 5000, 5);
[lm_err, left_Motor] = vrep.simxGetObjectHandle(vrep_id,'./leftMotor',vrep.simx_opmode_blocking);
[rm_err, right_Motor] = vrep.simxGetObjectHandle(vrep_id,'./rightMotor',vrep.simx_opmode_blocking);
[rp_err, robot_pose] = vrep.simxGetObjectHandle(vrep_id,'./robot_pose',vrep.simx_opmode_blocking);
[err, camhandle] = vrep.simxGetObjectHandle(vrep_id,'./VelodyneVPL16', vrep.simx_opmode_oneshot_wait);
[returnCode, SignalValue] = vrep.simxReadStringStream(vrep_id, 'datos', vrep.simx_opmode_streaming);

% SLAM
delta_t = 0.1; % Delta t
 % Number of obstacles/landmarks
radius_error = 0.3;


% SLAM RELATED

landmark_1 = [0,0];
landmark_2 = [0,0];
landmark_3 = [0,0];
landmark_4 = [0,0];
landmark_5 = [0,0];

init_x      = -4;
init_y      = 0;
init_theta  = pi/2;    % persona1: Radianes o Grados? persona2: si persona3: 🗣️🗣️🗣️🗣️🗣️🗣️🗣️🗣️🗣️🗣️🗣️🗣️ Radianes RAHHHHH !!!!!!!!!!!!!!!!!!!!! 🔥🔥🔥🔥🔥 PERSONA MENTIONED????????????????????????? 🗣️🗣️🗣️🗣️🗣️🗣️🗣️🗣️🔥🔥🔥🔥🔥🔥🔥

sigma_r     = 0.1;
sigma_phi   = 0.1;
N = 5; 
Q = [sigma_r 0; 0 sigma_phi];
R_t = diag([0.1, 0.1, 0.1]);

seen_landmark_mu = NaN(2,2*N);


init_pose   = [init_x, init_y, init_theta];


mu_0 = [init_pose(1), init_pose(2), init_pose(3), ...
        landmark_1(1), landmark_1(2), ...
        landmark_2(1), landmark_2(2), ...
        landmark_3(1), landmark_3(2), ...
        landmark_4(1), landmark_4(2), ...
        landmark_5(1), landmark_5(2), ...
        ];

dim = 2*N+3;
sigma_0 = zeros(dim, dim); % Crea la matriz de ceros
for i = 4:dim
    sigma_0(i, i) = 1000;
end


%**************************************************************************************************************************************************************
%***********************************************           MAIN          **************************************************************************************
%************************************************************************************************************************************************************** 


[x_path, y_path] = path_generator(radius, N_points); % circular path
[pose, v, w] = velocity_profiles(x_path, y_path, N_points, T_k); % velocity profiles generation

v_r = (L*w + 2*v)/(2); % right motor velocity
v_l = (2*v - L*w)/(2); % left  motor velocity
T_k = 0.1;
% Main loop
figure;

mu = mu_0';
sigma = sigma_0;

seen_correspondences = [0 ; 0]; % [radius, correspondence_value]

    
for i=1:N_points
    %clf; % Limpiar la figura para dibujar la nueva pose
    hold on;

    
    z_t = [0;0;0];
    
    [p_err, position] = vrep.simxGetObjectPosition(vrep_id, robot_pose,-1,vrep.simx_opmode_streaming);
    [o_err, orientation] = vrep.simxGetObjectOrientation(vrep_id, robot_pose,-1,vrep.simx_opmode_streaming);
    angle = orientation(3);

    if i==1
        x_real = -4;
        y_real = 0;
        angle = pi/2;
    else
        x_real = position(1);
        y_real = position(2);
    end
    
    %mu = [x_real, y_real,angle];

    if i ~= 1   
        
        point_cloud = get_lidar_measurement(vrep, vrep_id, camhandle);
        obstacles = Apply_DBScan(point_cloud);
        obstacles_centers = get_obstacle_centers(obstacles);
        z_t = get_z_from_obstacle_centers(obstacles_centers, mu);
        
        z_t = [z_t; zeros(1, size(z_t,2))]; % add new row for correspondence

        %disp("Num de obstaculos: ")
        %disp(size(obstacles_centers, 2))
        
        
        
        for radius_index =1:size(obstacles_centers, 2)
            radius = obstacles_centers(3, radius_index);
            c_i = -1;
            for c_index = 1:size(seen_correspondences,2)
                if abs(seen_correspondences(1,c_index)-radius) <= radius_error % Verify if seen feature has a correspondence already.
                    c_i = seen_correspondences(2,c_index);
                    fprintf("Correspondence already seen at %d\n",c_i);
                    z_t(3, radius_index) = c_i;
                    break;
                end
            end

            if c_i == -1 % If no correspondence was found
                new_c = [radius; seen_correspondences(2, end) + 1]; 
                seen_correspondences = [seen_correspondences, new_c]; % Add the new correspondence
                z_t(3, radius_index) = new_c(2,1);
            end

        end
        
        
        fprintf("seen obstacles: %d\n", size(obstacles_centers, 2))
        fprintf("seen correspondences: %d\n", size(seen_correspondences, 2)-1)
        
        colormap = ["red", "blue", "green", "black", "magenta"];

        if mod(i, 1) == 0
                %figure(floor(i/5));
                %hold on;
            for k = 1:size(obstacles)
                points = obstacles{k};
                scatter(points(1, :), points(2, :));
                scatter(mu(1,1), mu(2,1))
                % for lmk_i=1:N
                %     %text(mu(lmk_i + 3,1), mu(lmk_i + 4,1), num2str(lmk_i),'HorizontalAlignment', 'center','VerticalAlignment', 'middle', 'FontSize', 12, 'Color', 'r');
                %     scatter(mu(2+2*lmk_i,1), mu(3+2*lmk_i,1), 100 ,colormap(lmk_i), 'd')
                % end
                
            end
            viscircles(obstacles_centers(1:2,:)', obstacles_centers(3,:)');
        end
    end

    x_acum(i) = x_real;
    y_acum(i) = y_real;
    theta_acum(i) = angle;
    min_dist = inf;

    % Nearest point to the robot
    for j = 1:length(x_path)-1
        dist_temp = sqrt((x_real - x_path(j))^2 + (y_real - y_path(j))^2);
        if dist_temp < min_dist
            min_dist = dist_temp;
            x_min = x_path(j);
            y_min = y_path(j);
        end
    end

    % Project x and y to the robot local frame
    x_local = (x_min - x_real)*cos(angle) + (y_min-y_real)*sin(angle);
    y_local = -(x_min - x_real)*sin(angle) + (y_min-y_real)*cos(angle);

    x_acum_local(i) = x_local;
    y_acum_local(i) = y_local;
    x_min_acum(i) = x_min;
    y_min_acum(i) = y_min;

    pid = Kp*y_local + Ki*(acumulated_error + y_local); % steer correction
    acumulated_error = acumulated_error + y_local;
    
    u_t = [v(i), w(i)];
    
    if i ~= 1
        [mu_t,sigma_t, updated_lmk] = execute_EKFSLAM(mu, sigma, u_t, z_t, seen_landmark_mu, R_t, Q);
        
        mu = mu_t;
        sigma = sigma_t;
        seen_landmark_mu = updated_lmk;
    end

    [rm_err] = vrep.simxSetJointTargetVelocity(vrep_id,right_Motor,v_r(i) + pid,vrep.simx_opmode_oneshot );
    [lm_err] = vrep.simxSetJointTargetVelocity(vrep_id,left_Motor,v_l(i) - pid,vrep.simx_opmode_oneshot );


    pause(3.1*T_k)
end

[rm_err] = vrep.simxSetJointTargetVelocity(vrep_id,right_Motor,0,vrep.simx_opmode_oneshot );
[lm_err] = vrep.simxSetJointTargetVelocity(vrep_id,left_Motor,0,vrep.simx_opmode_oneshot );

%lin_vel_test = 0.1;
%ang_vel_test = 0.1;
%[mu_bar, sigma_bar] = EKFSLAM_prediction(mu_0, sigma_0, lin_vel_test, ang_vel_test, N, delta_t, R);
%mu_test = [1 1 pi/4 1 1 2 2 3 3 4 4 5 5];
%z_test = [1 2 3 4 5; 9 8 7 6 5];
%c_test = [1 2 3 4 5];
%a = EKFSLAM_correction(mu_bar, sigma_bar, seen_landmark_mu ,z_test, c_test, Q);

%***********************************************************************************************************************************************************
%***********************************************      FUNCIONES       **************************************************************************************
%***********************************************************************************************************************************************************

function [x_path, y_path] = path_generator(radius, N)
    angle = linspace(0, 2*pi, N); 
    x_path = -radius * cos(angle); 
    y_path = radius * sin(angle); 
end

function [pose, v, w] = velocity_profiles(x_path, y_path, N, T_k)
    v_i = zeros(1, N); % Vector de velocidades lineales
    w_i = zeros(1, N); % Vector de velocidades angulares
    
    % Theta generator
    theta = zeros(1,N);
    for i=1:N
        if i==N
            theta(i) = theta(i-1);
        else
            theta(i) = atan( (y_path(i+1) - y_path(i)) / (x_path(i+1) - x_path(i)));
        end
    end
    theta(N/2+1:end) = theta(N/2+1:end) - pi;
    pose = [x_path;y_path;theta];
    % Generación de perfiles de velocidad
    for i=1:N
        if i==N
            v(i) = 0;
            w(i) = 0;
        else
        A = [cos(theta(i)) 0; sin(theta(i)) 0; 0 1];
        A_inv = pinv(A);
        velocities = 1/T_k.*A_inv*(pose(:,i+1)- pose(:,i));
        v_i(i) = velocities(1,:);
        w_i(i) = velocities(2,:);
        end
    end 
    v = v_i;
    w = w_i;
end

function [point_cloud] = get_lidar_measurement(vrep, vrep_id, camhandle)  %point_cloud esta en formato [x, y, z]
    full_vect = zeros(1,1);
    for n = linspace(1, 1, 1)
        [returnCode, SignalValue] = vrep.simxReadStringStream(vrep_id, 'datos', vrep.simx_opmode_buffer);
        RealValue = vrep.simxUnpackFloats(SignalValue);
        full_vect = cat(2, full_vect, RealValue);
        pause(0.1)
    end
    %k = 3*10000;
    full_vect = full_vect(2:end);
    M = length(full_vect) / 3;
    point_cloud = reshape(full_vect, [3, M]);
end

function [obstacles_centers] = get_obstacle_centers(obstacles)
    for n_cluster = 1:size(obstacles)
        obstacle_point_cloud = obstacles{n_cluster};
        x = obstacle_point_cloud(1, :);
        y = obstacle_point_cloud(2, :);
    
        Y = x.^2 + y.^2;
        X_1 = x;
        X_2 = y;
        X = [2*X_1', 2*X_2', ones(size(x, 2), 1)];
    
        e(:, n_cluster) = regress(Y', X);
        obstacles_centers(:, n_cluster) = [e(1, n_cluster), e(2, n_cluster), sqrt(e(1, n_cluster)^2 + e(2, n_cluster)^2 + e(3, n_cluster))];
    end
end

function [z_t] = get_z_from_obstacle_centers(obstacle_centers, mu_bar)
    num_features = size(obstacle_centers, 2);
    z_t = zeros(2, num_features);
    for k = 1:num_features
    z_r = sqrt((obstacle_centers(1, k)-mu_bar(1))^2+(obstacle_centers(2, k)-mu_bar(2))^2);
    z_phi = atan2(obstacle_centers(2, k)-mu_bar(2), obstacle_centers(1, k)-mu_bar(1)) - mu_bar(3);
    z_t(1,k) = z_r;
    z_t(2,k) = z_phi;
    %z_t(k) = [z_r; z_phi];
    end
end

function [mu_t,sigma_t, updated_lmk] = execute_EKFSLAM(mu_t_1, sigma_t_1, u_t, z_t, lmk, R_t, Q)
    disp("Performing EKF, trust me"); %SACAR ESTO, o no, da lo mismo en realidad
    % Covariance Matrix Q and R
    %R = eye(size(mu_t_1));  % Covariance of motion
    %Q = eye(size(z_t));     % Covariance of observation
    N = 5;
    delta_t = 0.1;

    % Prediction
    [mu_bar, sigma_bar] = EKFSLAM_prediction(mu_t_1,sigma_t_1,u_t(1), u_t(2), N, delta_t, R_t);
    
    %mu_bar
    %mu_t = mu_bar;
    %sigma_t = sigma_bar;
    % Correction
    [mu_t, sigma_t, updated_lmk] = EKFSLAM_correction(mu_bar, sigma_bar, lmk, z_t, Q );

end

function [mu_bar, sigma_bar] = EKFSLAM_prediction(mu, sigma, linear_velocity, angular_velocity, N, delta_t, R)
    % Ver como hacemo esto
    
    F_x = [ eye(3,3) , zeros(3, 2*N)];
    model = [-linear_velocity/angular_velocity*sin(mu(3)) + linear_velocity/angular_velocity*sin(mu(3) + angular_velocity*delta_t); ...
             linear_velocity/angular_velocity*cos(mu(3)) - linear_velocity/angular_velocity*cos(mu(3) + angular_velocity*delta_t); ...
             angular_velocity*delta_t];
    

    mu_bar = mu + F_x'*model;
    
    % Derivation of G_t
    
    Gtx = eye(3,3);
    Gtx(1,3) = -model(2);
    Gtx(2,3) =  model(1);
    
    I = eye(2*N);
    Gt = zeros(size(Gtx) + size(I));
    Gt(1:size(Gtx,1), 1:size(Gtx,2)) = Gtx;
    Gt(size(Gtx,1)+1:end, size(Gtx,2)+1:end) = I;
    
    sigma_bar = Gt * sigma * Gt' + F_x'*R*F_x;

end

function [corrected_mu, corrected_sigma, seen_landmark_mu] = EKFSLAM_correction(mu_bar, sigma_bar, seen_landmark_mu, z_t, Q)
    if z_t ~= [0;0;0]
        N = 5;
        mu = mu_bar;
        sigma = sigma_bar;
        [rows , cols] = size(z_t);
        
        for feature = 1:cols
            range = z_t(1,feature);
            bear  = z_t(2,feature);
            j     = z_t(3,feature); % c_t^i
            
            fprintf("For feature %d, j = %d \n", feature, j)
    
            if isnan(seen_landmark_mu(1,j))
                seen_landmark_mu(1,j)   = mu_bar(1) + range*cos(bear + mu_bar(3));
                seen_landmark_mu(2,j)   = mu_bar(2) + range*sin(bear + mu_bar(3));
            end
    
            mu_jx = seen_landmark_mu(1,j);
            mu_jy = seen_landmark_mu(2,j);
            
            delta_x = mu_jx - mu_bar(1);
            delta_y = mu_jy - mu_bar(2);
            delta = [delta_x ; delta_y];
            q = delta'*delta;
            z_hat = [sqrt(q) ; atan2(delta_y,delta_x)- mu_bar(3)];
            
    
            F_xj = zeros(5, 5 + 2*N-2);
            F_xj(1,1) = 1;
            F_xj(2,2) = 1;
            F_xj(3,3) = 1;
    
            F_xj(4, 2*j+2) = 1;
            F_xj(5, 2*j+3) = 1;
            
            H_t = 1/q*[ -sqrt(q)*delta_x, -sqrt(q)*delta_y, 0 , sqrt(q)*delta_x, sqrt(q)*delta_y; delta_y , -delta_x, -q, -delta_y, delta_x]*F_xj;
    
            K_t = sigma_bar*H_t'*pinv(H_t*sigma_bar*H_t' + Q);
            zt = z_t(1:2,feature);
            
            % disp("Shape of zt")
            % disp(size(zt))
            % 
            % disp("Shape of mu")
            % disp(size(mu))
            % 
            % disp("Shape of Kt")
            % disp(size(K_t))
            % 
            % disp("Shape of z_hat")
            % disp(size(z_hat))
            % 
            % aa = K_t*(zt - z_hat);
            % 
            % disp("Shape of the other thing")
            % disp(size(aa))

            mu = mu + K_t*(zt - z_hat);
            KH = K_t*H_t;
            sigma = (eye(size(KH)) - KH)*sigma;
        end
    
        corrected_mu    = mu;
        corrected_sigma = sigma;
    else
        corrected_mu    = mu_bar;
        corrected_sigma = sigma_bar;
    end
end

function [obstacles] = Apply_DBScan(point_cloud)

    x = point_cloud(1,:); y = point_cloud(2,:); z = point_cloud(3,:);
    
    floor_threshold = min(z) + 0.6;
    non_floor_idx = z > floor_threshold;

    filtered_x = x(non_floor_idx);
    filtered_y = y(non_floor_idx);

    % Clustering Points via DBSCAN
    epsilon = 0.1;  % Max distance between points on the same cluster
    minpts = 3;
    idx = dbscan([filtered_x',filtered_y'], epsilon, minpts);

    % Plot points (OPTIONAL)
    %figure;
    %scatter(filtered_x, filtered_y, 10, idx, 'filled');
    %xlabel('X [m]');
    %ylabel('Y [m]');
    %title('DBSCAN Results');
    %colorbar;

    % Create obstacle polygons
    unique_clusters = unique(idx(idx > 0)); % Ignore Noise (Cluster with a '0' label)
    
    %fprintf('Quantity of unique clusters: %d\n', length(unique_clusters));
    if isempty(unique_clusters)
        fprintf('No clusters found\n');
    end
    
    obstacles = cell(length(unique_clusters), 1);

    for j = 1:length(unique_clusters)
        cluster_points = [filtered_x(idx == unique_clusters(j)); filtered_y(idx == unique_clusters(j))];

        if size(cluster_points, 2) >= 70
            obstacles{j} = cluster_points;
        else
            fprintf('Cluster No %d does not have enough points to make a polygon\n', unique_clusters(j));
        end
    end

    % Delete empty obstacles
    obstacles = obstacles(~cellfun('isempty',obstacles));
end