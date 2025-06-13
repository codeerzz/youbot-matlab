% ROS'tan odometri verisini al
subOdom = rossubscriber('/odom', 'nav_msgs/Odometry');
odomMsg = receive(subOdom, 10);

% x, y ve yaw değerlerini çıkar
x_init = odomMsg.Pose.Pose.Position.X;
y_init = odomMsg.Pose.Pose.Position.Y;

% Quaternion'dan Yaw açısını hesapla
quat = odomMsg.Pose.Pose.Orientation;
yaw_init = quat2eul([quat.W quat.X quat.Y quat.Z]); % [roll, pitch, yaw] dönüşümü
yaw_init = yaw_init(1); % Sadece yaw açısını al

% Başlangıç pozisyonu vektörü
q_initi = [x_init; y_init; yaw_init]