
%Ros init atmak lazım en başta
tftree = rostf;

% Haritayı al
subMap = rossubscriber('/map', 'nav_msgs/OccupancyGrid');
msg = receive(subMap);
msgStruct = struct(msg); % Mesajı struct formatına dönüştür
map = rosReadOccupancyGrid(msgStruct); % Occupancy grid haritasını oluştur

% Robotun başlangıç konumunu ROS'tan al
tfData = getTransform(tftree, 'map', 'base_link', 'Timeout', 5);


% Dönüşümden konum bilgisi (x, y, z) çek
pos = tfData.Transform.Translation;
xRobot = pos.X;
yRobot = pos.Y;
zRobot = pos.Z;  % 2D plan için çoğunlukla 0 olarak kabul edilir ama yine de çekilebilir

% Dönüşümden yönelim (quaternion) bilgisi çek
quat = tfData.Transform.Rotation;
% MATLAB'da quaternion -> Euler (RPY) dönüşümü: 
eul = quat2eul([quat.W quat.X quat.Y quat.Z]);  % [roll pitch yaw] formatında gelir
yaw = eul(3);  % RRT planlamak için genelde yaw ihtiyacı olur

% Robotun [x, y, theta] konumu
robotPose = [xRobot, yRobot, yaw];
% --- ODOM yerine TF'den Konum Alımı Tamam ---

figure;
show(map)
zoom on;
hold on;
% Başlangıç konumunu haritada göster
scatter(robotPose(1), robotPose(2), 'go', 'filled'); % Yeşil daire başlangıç konumunu temsil eder
disp(['Mevcut konum: X=', num2str(robotPose(1)), ', Y=', num2str(robotPose(2))]);
zoom on; % Zoom özelliğini etkinleştir
disp('Lütfen zoom yapıp, devam etmek için bir tuşa basın...');
pause; % Kullanıcı zoom yaparken bekler
zoom off; % Zoom'u devre dışı bırak

% Kullanıcının haritada bir nokta seçmesi
[xGoal, yGoal] = ginput(1); % Kullanıcı tıklamasıyla bir nokta seçer

% Seçilen konumu hedef olarak ayarla
goal = [xGoal, yGoal, 0]; % Z ekseni genelde 0 kabul edilir


% Hedef konumu haritada göster
scatter(xGoal, yGoal, 50, 'r', 'filled'); % Kırmızı daire hedefi temsil eder
disp(['Seçilen hedef konumu: X=', num2str(xGoal), ', Y=', num2str(yGoal)]);



% Gerçek dünya koordinatlarını grid indekslerine dönüştür
%startGrid = world2grid(map, robotPose);
%goalGrid = world2grid(map, goal);


ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 0.01;
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits;[-pi pi]];



% RRT planlayıcı oluştur ve yol planla
planner = plannerRRT(ss,sv,MaxConnectionDistance=0.3);
planner.MaxConnectionDistance = 0.3;
planner.MaxIterations = 1000;

[pathObj, solInfo] = plan(planner, robotPose, goal);

if solInfo.IsPathFound
    disp('RRT algoritması başarılı bir yol buldu!');
    plot(pathObj.States(:,1), pathObj.States(:,2), 'r', 'LineWidth', 2); % Yol

    % Planı /move_base/NavfnROS/plan topic’ine gönder
    planPub = rospublisher('/move_base/NavfnROS/plan', 'nav_msgs/Path'); % Yayıncı oluştur
    pathMsg = rosmessage(planPub); % nav_msgs/Path mesajı oluştur

    % Header bilgilerini doldur
    pathMsg.Header.FrameId = 'map'; % Harita çerçevesini kullan
    pathMsg.Header.Stamp = rostime('now'); % Zaman damgası
    waypoint = zeros(size(pathObj.States, 1),2);
    % map → odom dönüşümünü al (sadece translasyon)
    map2odom_tf = getTransform(tftree, 'odom', 'map', 'Timeout', 3);
    dx = map2odom_tf.Transform.Translation.X;
    dy = map2odom_tf.Transform.Translation.Y;
    % Yol noktalarını doldur
    for i = 1:size(pathObj.States, 1)
        pose = rosmessage('geometry_msgs/PoseStamped'); % Her yol noktası için mesaj
        pose.Header.FrameId = 'map';
        pose.Pose.Position.X = pathObj.States(i,1);
        pose.Pose.Position.Y = pathObj.States(i,2);
        pose.Pose.Position.Z = 0; % 2D plan için Z=0
        pose.Pose.Orientation.W = 1.0; % Yön (varsayılan olarak hiçbir döndürme yok
           
        waypoint(i,1) = pathObj.States(i,1) + dx;
        waypoint(i,2) =pathObj.States(i,2)+ dy;

        pathMsg.Poses(i) = pose; % Mesajı listeye ekle
    end

    % Mesajı yayınla
    send(planPub, pathMsg);
    disp('Plan /move_base/NavfnROS/plan topic’ine gönderildi!');
else
    disp('RRT algoritması bir yol bulamadı.');
end