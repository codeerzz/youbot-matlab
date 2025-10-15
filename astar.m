
% TF (Transform) ağacına erişim için nesne oluştur
tftree = rostf;

% Haritayı almak için '/rtabmap/grid_map' topic'ine abone ol
subMap = rossubscriber('/rtabmap/grid_map', 'nav_msgs/OccupancyGrid');
msg = receive(subMap);
map = rosReadOccupancyGrid(msg); % OccupancyGrid mesajını MATLAB harita nesnesine çevir

% --- Robotun Başlangıç Konumunu TF'den Alma ---
% 'map' ve 'base_link' frameleri arasındaki dönüşümü al
tfData = getTransform(tftree, 'map', 'base_link', 'Timeout', 5);

% Konum bilgisi (x, y)
pos = tfData.Transform.Translation;
xRobot = pos.X;
yRobot = pos.Y;

% Yönelim bilgisi (quaternion -> Euler)
quat = tfData.Transform.Rotation;
eul = quat2eul([quat.W quat.X quat.Y quat.Z]);
yaw = eul(3); % Sadece yaw (dönüş) açısını alıyoruz

% Robotun başlangıç pozisyonu [x, y, theta]
robotPose = [xRobot, yRobot, yaw];

% --- GÖRSELLEŞTİRME VE HEDEF SEÇİMİ ---
figure;
show(map);
hold on;
title('A* Path Planning');

% Başlangıç konumunu haritada göster (Yeşil daire)
scatter(robotPose(1), robotPose(2), 50, 'g', 'filled', 'DisplayName', 'Başlangıç');
disp(['Mevcut konum: X=', num2str(robotPose(1)), ', Y=', num2str(robotPose(2))]);

% Kullanıcının haritadan bir hedef nokta seçmesi
zoom on;
disp('Lütfen haritada bir hedef noktası seçmek için tıklayın...');
[xGoal, yGoal] = ginput(1);
zoom off;

% Hedef konumu haritada göster (Kırmızı daire)
scatter(xGoal, yGoal, 50, 'r', 'filled', 'DisplayName', 'Hedef');
legend;
disp(['Seçilen hedef konumu: X=', num2str(xGoal), ', Y=', num2str(yGoal)]);
hold off;

% --- A* İLE YOL PLANLAMA BÖLÜMÜ ---

% ÖNEMLİ: A* algoritmasının güvenli bir yol bulması için haritadaki engelleri
% robotun yarıçapı kadar şişirmek (inflate) iyi bir pratiktir.
% Robotunuzun yarıçapına göre bu değeri ayarlayın.
robotRadius = 0.3; % Örnek: 20 cm yarıçap
inflatedMap = copy(map);
inflate(inflatedMap, robotRadius);

% A* planlayıcısı için başlangıç ve hedef konumlarını dünya koordinatlarından
% haritanın grid (ızgara) indekslerine dönüştür.
startGrid = world2grid(inflatedMap, [robotPose(1), robotPose(2)]);
goalGrid = world2grid(inflatedMap, [xGoal, yGoal]);

% A* planlayıcı nesnesini şişirilmiş harita ile oluştur
planner = plannerAStarGrid(inflatedMap);

% Planlamayı gerçekleştir. Çıktı, grid hücrelerinin indeksleridir.
[pathGrid, ~] = plan(planner, startGrid, goalGrid);

% Planlama sonucunu kontrol et
if ~isempty(pathGrid)
    disp('A* algoritması başarılı bir yol buldu!');
    
    % Bulunan grid yolunu tekrar dünya koordinatlarına çevir
    pathWorld = grid2world(inflatedMap, pathGrid);
    
    % Yolu harita üzerinde göster
    hold on;
    plot(pathWorld(:,1), pathWorld(:,2), 'b', 'LineWidth', 2, 'DisplayName', 'A* Yolu');
    hold off;
    
    % --- Planı ROS Topic'ine Yayınlama ---
    planPub = rospublisher('/move_base/NavfnROS/plan', 'nav_msgs/Path');
    pathMsg = rosmessage(planPub);
    
    % Header bilgilerini doldur
    pathMsg.Header.FrameId = 'map';
    pathMsg.Header.Stamp = rostime('now');
    waypoint = zeros(size(pathObj.States, 1),2);
    % Yol noktalarını (waypoint) path mesajına ekle
    for i = 1:size(pathWorld, 1)
        pose = rosmessage('geometry_msgs/PoseStamped');
        pose.Header.FrameId = 'map';
        pose.Header.Seq = i-1; % Sıra numarası
        pose.Header.Stamp = rostime('now');
        
        % Konum bilgilerini ayarla
        pose.Pose.Position.X = pathWorld(i,1);
        pose.Pose.Position.Y = pathWorld(i,2);
        pose.Pose.Position.Z = 0; % 2D planlama için Z=0
        
        % Yönelim (Orientation) - A* yönelim bilgisi vermez, bu yüzden
        % her noktada yönelimi şimdilik sabit tutuyoruz. Gerçek bir robot
        % kontrolcüsü bu yolu takip ederken yönelimi kendisi ayarlar.
        pose.Pose.Orientation.W = 1.0;
        pose.Pose.Orientation.X = 0.0;
        pose.Pose.Orientation.Y = 0.0;
        pose.Pose.Orientation.Z = 0.0;
        waypoint(i,1) = pathObj.States(i,1) ;
        waypoint(i,2) =pathObj.States(i,2);
           
        pathMsg.Poses(i) = pose; % Oluşturulan pozu yola ekle
    end
    
    % Mesajı yayınla
    send(planPub, pathMsg);
    disp('Plan /move_base/NavfnROS/plan topic'ine gönderildi!');
    
else
    disp('A* algoritması bir yol bulamadı. Hedef ulaşılamaz veya harita dışında olabilir.');
end

% ROS bağlantısını kapat
rosshutdown;