%% YouBot Arm Visualization with End-Effector Position Display
% Bu kod, joints: [2.3061 0.4166 -0.1351 1.6646 1.3526] için KUKA YouBot
% kolunun 3B görselini oluşturur ve end-effector pozisyonunu gösterir.
joint_offsets = [2.94961, 2.70526, -2.54818, 3.36174, 2.92343];
%joint_angles = [    1,-0.6,-0.3,-0.5,0];
%joint_angles = [    -1,-0.6,0.7,-0.5,0];
% Verilen eklem açılar (radyan cinsinden)
%joint_angles = [ 0.8038,0.1466,-0.5687,-0.6998,   0.0000];
%joint_angles = [    -0.7697 ,   0.7060,   -0.1749 ,   0.7439  , -0.7916];
%joint_angles = [    1,1,0,0,0];
%joint_angles = [    0,0,0,0,0];
%joint_angles = [     706.3391, 100.2198 ,   -5.8604,     31.1567 ,0.0000];
joint_angles = [0, 0.8, -1.2, 0.9, 0] - joint_offsets;



% KUKA YouBot için DH parametreleri (her satır: [a, alpha, d, theta])
% Burada theta, her eklem için verilen joint_angles ile toplanır.
DH_params = [ 0.033,      pi/2, 0.147, joint_angles(1)+2.94961;
             0.155,    0,    0,     joint_angles(2)+2.70526;
             0.135,    0,    0,     joint_angles(3)-2.54818;
             0,       pi/2, 0,  joint_angles(4)+3.36174;
             0,        0,    0.2174,joint_angles(5)+2.92343];


num_joints = 5;
% Base (0. eklem) pozisyonu
points = zeros(3, num_joints+1);  % Base + 5 eklem
points(:,1) = [0; 0; 0];  % Base orijini

T = eye(4);
for i = 1:num_joints
    % Her eklem için D-H dönüşümünü hesapla ve sırayla çarp
    T = T * dh_matrix(DH_params(i,1), DH_params(i,2), DH_params(i,3), DH_params(i,4));
    % Homojen dönüşüm matrisinden (x,y,z) pozisyon bilgisini al
    points(:,i+1) = T(1:3,4);
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
plot3(points(1,:), points(2,:), points(3,:), 'b-o', 'LineWidth', 2, 'MarkerSize', 8);

% Base koordinat sistemi için oklar (X: kırmızı, Y: yeşil, Z: mavi)
quiver3(0, 0, 0, 0.1, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver3(0, 0, 0, 0, 0.1, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver3(0, 0, 0, 0, 0, 0.1, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% Her eklem noktasını etiketleyelim
for i = 1:size(points,2)
    text(points(1,i), points(2,i), points(3,i), sprintf('  Joint %d', i-1), 'FontSize', 9, 'Color', 'm');
end

% End-effector pozisyonunu daha belirgin hale getirmek için farklı renk ve işaretleyici kullan
plot3(ee_pos(1), ee_pos(2), ee_pos(3), 'kp', 'MarkerSize', 15, 'MarkerFaceColor', 'r'); 
text(ee_pos(1), ee_pos(2), ee_pos(3), '  End-Effector', 'FontSize', 10, 'Color', 'r');

% EE pozisyonunu konsola yazdır
fprintf('End-Effector Pozisyonu: X = %.4f, Y = %.4f, Z = %.4f\n', ee_pos(1), ee_pos(2), ee_pos(3));

view(3);
legend('Robot Kolu', 'X-axis', 'Y-axis', 'Z-axis', 'End-Effector');

%% --- Yardımcı Fonksiyon ---
function T = dh_matrix(a, alpha, d, theta)
    % DH parametrelerine göre 4x4 dönüşüm matrisini hesaplar.
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,          sin(alpha),            cos(alpha),             d;
         0,          0,                     0,                      1];
end
