function waypoints = rrtPathPlanning()
    % Haritayı al
    subMap = rossubscriber('/map', 'nav_msgs/OccupancyGrid');
    msg = receive(subMap, 10); % Mesajı al (timeout 10 saniye)
    msgStruct = struct(msg); % Mesajı struct formatına dönüştür
    map = rosReadOccupancyGrid(msgStruct); % Occupancy grid haritasını oluştur
    show(map);

    % Robotun başlangıç konumunu al
    subOdom = rossubscriber('/odom', 'nav_msgs/Odometry');
    odomMsg = receive(subOdom, 10);
    robotPose = [odomMsg.Pose.Pose.Position.X, odomMsg.Pose.Pose.Position.Y, ...
                 odomMsg.Pose.Pose.Orientation.Z];

    % Hedef konum
    goal = [0.403, 1.977, odomMsg.Pose.Pose.Position.Z]; % Manuel hedef koordinatları

    % State space ve validatör oluştur
    ss = stateSpaceSE2;
    sv = validatorOccupancyMap(ss);
    sv.Map = map;
    sv.ValidationDistance = 0.01;
    ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

    % RRT planlayıcıyı oluştur
    planner = plannerRRT(ss, sv, MaxConnectionDistance=0.3);
    planner.MaxConnectionDistance = 0.3;
    planner.MaxIterations = 1000;

    % RRT algoritmasını çalıştır
    [pathObj, solInfo] = plan(planner, robotPose, goal);

    if solInfo.IsPathFound
        disp('RRT algoritması başarılı bir yol buldu!');
        
        % Planlanan yolu görselleştir
        figure;
        show(map);
        hold on;
        scatter(robotPose(1), robotPose(2), 'go', 'filled'); % Başlangıç
        scatter(goal(1), goal(2), 'bo', 'filled'); % Hedef
        plot(pathObj.States(:,1), pathObj.States(:,2), 'r', 'LineWidth', 2); % Yol
        title('RRT Planlanan Yol');
        hold off;

        % Waypoint'leri array olarak döndür
        waypoints = pathObj.States(:, 1:2); % Sadece X ve Y koordinatları
    else
        disp('RRT algoritması bir yol bulamadı.');
        waypoints = []; % Yol bulunamadığında boş array döndür
    end

end
