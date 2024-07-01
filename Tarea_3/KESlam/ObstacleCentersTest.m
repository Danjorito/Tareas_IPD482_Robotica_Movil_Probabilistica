%%
% Inicializacion de la API
clc; clear; close all;

vrep = remApi('remoteApi');
vrep.simxFinish(-1);
vrep_id = vrep.simxStart('127.0.0.1', 19000, true, true, 5000, 5);

[err, camhandle] = vrep.simxGetObjectHandle(vrep_id,'./VelodyneVPL16', vrep.simx_opmode_oneshot_wait);
[returnCode, SignalValue] = vrep.simxReadStringStream(vrep_id, 'datos', vrep.simx_opmode_streaming);




point_cloud = get_lidar_measurement(vrep, vrep_id, camhandle);
disp(length(point_cloud));
obstacles = Apply_DBScan(point_cloud);
centros = get_obstacle_centers(obstacles);
pause(1);


figure(3)
hold on;
for k = 1:size(obstacles)
    points = obstacles{k};
    scatter(points(1, :), points(2, :));
end

viscircles(centros(1:2,:)', centros(3,:)');

function [point_cloud] = get_lidar_measurement(vrep, vrep_id, camhandle)  %point_cloud esta en formato [x, y, z]
    full_vect = zeros(1,1);
    for n = linspace(1, 4, 4)
        [returnCode, SignalValue] = vrep.simxReadStringStream(vrep_id, 'datos', vrep.simx_opmode_buffer);
        RealValue = vrep.simxUnpackFloats(SignalValue);
        full_vect = cat(2, full_vect, RealValue);
        pause(0.1)
    end
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