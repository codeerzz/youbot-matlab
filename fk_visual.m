%% YouBot Arm Visualization with End-Effector Position Display
% Bu kod, joints: [2.3061 0.4166 -0.1351 1.6646 1.3526] için KUKA YouBot
% kolunun 3B görselini oluşturur ve end-effector pozisyonunu gösterir.
%joint_offsets = [2.94961, 2.60, -2.54818, 3.36174, 2.92343];

joint_offsets = [2.9496, 1.1344, -2.5481, 1.7889, 2.9234];
joint_angles = [0,0,0,0,0];%matlab kinematik görselleştirme
joint_angles_pub = joint_angles+[joint_offsets]%youbota publish edilecek değerler

% KUKA YouBot için DH parametreleri (her satır: [a, alpha, d, theta])
DH_params = [0.033, pi/2, 0.147, -joint_angles(1) ;
             0.155, 0, 0, joint_angles(2)+pi/2 ;
             0.135, 0, 0, joint_angles(3) ;
             0, pi/2, 0, joint_angles(4)+pi/2 ;
             0, 0, 0.187, -joint_angles(5) ];

num_joints = size(DH_params, 1);
points = zeros(3, num_joints + 1);  % Base + 5 eklem
T = eye(4);

% Dönüşüm matrislerini hesapla ve eklem noktalarını güncelle
for i = 1:num_joints
    T = T * dh_matrix(DH_params(i, :));
    points(:, i + 1) = T(1:3, 4);
end

% End-effector pozisyonu son hesaplanan nokta (points(:,end))
ee_pos = points(:, end);

% 3B çizim ayarları
figure;
hold on;
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('KUKA YouBot Arm Visualization');

% Robot kolu: eklem noktalarını birleştir ve çiz (mavi çizgi ve işaretleyiciler)
plot3(points(1, :), points(2, :), points(3, :), 'b-o', 'LineWidth', 2, 'MarkerSize', 8);

% Base koordinat sistemi için oklar (X: kırmızı, Y: yeşil, Z: mavi)
quiver3(0, 0, 0, 0.1, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver3(0, 0, 0, 0, 0.1, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver3(0, 0, 0, 0, 0, 0.1, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% Her eklem noktasını etiketleyelim
for i = 1:num_joints + 1
    text(points(1, i), points(2, i), points(3, i), sprintf('  Joint %d', i - 1), 'FontSize', 9, 'Color', 'm');
end

% End-effector pozisyonunu daha belirgin hale getirmek için farklı renk ve işaretleyici kullan
plot3(ee_pos(1), ee_pos(2), ee_pos(3), 'kp', 'MarkerSize', 15, 'MarkerFaceColor', 'r'); 
text(ee_pos(1), ee_pos(2), ee_pos(3), '  End-Effector', 'FontSize', 10, 'Color', 'r');

% EE pozisyonunu konsola yazdır
fprintf('End-Effector Pozisyonu: X = %.4f, Y = %.4f, Z = %.4f\n', ee_pos(1), ee_pos(2), ee_pos(3));

view(3);
legend('Robot Kolu', 'X-axis', 'Y-axis', 'Z-axis', 'End-Effector');

%% --- Yardımcı Fonksiyon ---
function T = dh_matrix(params)
    % DH parametrelerine göre 4x4 dönüşüm matrisini hesaplar.
    a = params(1);
    alpha = params(2);
    d = params(3);
    theta = params(4);
    T = [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta);
         sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end
