% ROS subscriber'ları tanımla
subMap = rossubscriber('/map', 'nav_msgs/OccupancyGrid');
subOdom = rossubscriber('/odom', 'nav_msgs/Odometry');
subPath = rossubscriber('/move_base/NavfnROS/plan', 'nav_msgs/Path');

% İlk haritayı al ve göster
msgStruct = struct(msg); % Mesajı struct formatına dönüştür
map = rosReadOccupancyGrid(msgStruct); % Haritayı OccupancyGrid formatına çevir
figure;
ax = show(map); % Haritayı çiz
axis equal;
hold on;

% Görselleştirme döngüsü
while true
    % Haritayı sürekli al (isteğe bağlı, genelde sabittir)
    if subMap.LatestMessage
        msgMap = subMap.LatestMessage;
        map = rosReadOccupancyGrid(msgMap);
        show(map);
    end
    
    % Robotun pozisyonunu al
    if subOdom.LatestMessage
        msgOdom = subOdom.LatestMessage;
        robotPose = [msgOdom.Pose.Pose.Position.X, msgOdom.Pose.Pose.Position.Y];
        scatter(robotPose(1), robotPose(2), 50, 'go', 'filled'); % Robot pozisyonunu işaretle
    end
    
    % Planlanan yolu al
    if subPath.LatestMessage
        msgPath = subPath.LatestMessage;
        poses = msgPath.Poses;
        pathPoints = zeros(length(poses), 2);
        for i = 1:length(poses)
            pathPoints(i, :) = [poses(i).Pose.Position.X, poses(i).Pose.Position.Y];
        end
        plot(pathPoints(:, 1), pathPoints(:, 2), 'r-', 'LineWidth', 2); % Planlanan yolu çiz
    end
    
    % Çizimi güncelle
    drawnow; % Görselleştirmeyi sürekli günceller
    
    % Döngüyü durdurmak için bir tuşa basabilirsiniz
    if ~ishandle(ax)
        break; % Figür kapatıldıysa döngüden çık
    end
end