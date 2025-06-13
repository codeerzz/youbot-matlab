function kuka_youbot_ik
    % KUKA YouBot için İnvers Kinematik (IK)
    % Hedef end-effector pozisyonu (metre cinsinden)
    target_position = [1.2; 0.0; 0.0]; 
    target_pose = eye(4);
    target_pose(1:3,4) = target_position;
    
    % Başlangıç eklem açıları (radyan cinsinden) - 5x1 sıfır vektörü
    initial_thetas = zeros(5,1);
    
    % İnvers Kinematik çözümünü hesapla
    solution = inverse_kinematics(target_pose, initial_thetas);
    
    % Çözülen eklem açılarını ve elde edilen end-effector pozisyonunu göster
    disp('Çözülen eklem açıları (radyan cinsinden):');
    disp(solution);
    
    end_effector_pose = forward_kinematics(solution);
    disp('Elde edilen son end-effector pozisyonu:');
    disp(end_effector_pose(1:3,4));
end

% D-H parametrelerine göre dönüşüm matrisi hesaplama fonksiyonu
function T = dh_matrix(a, alpha, d, theta)
    % a: link uzunluğu (m)
    % alpha: link bükme açısı (radyan)
    % d: link offset (m)
    % theta: eklem açısı (radyan)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end

% İleri kinematik hesaplama: Tüm eklemlerin dönüşüm matrisleri çarpılarak end-effector konumunu ve oryantasyonunu verir.
function T = forward_kinematics(thetas)
    % thetas: 5 eklem açısı (5x1 vektör)
    % D-H Parametreleri: [a, alpha, d, theta] (her bir satır bir eklemi temsil eder)
DH_params = [ 0.033,      pi/2, 0.147, thetas(1)+2.94961;
             0.155,    0,    0,     thetas(2)+2.70526;
             0.135,    0,    0,     thetas(3)-2.54818;
             0,       pi/2, 0,  thetas(4)+3.36174;
             0,        0,    0.2174,thetas(5)+2.92343];
    T = eye(4);
    for i = 1:size(DH_params,1)
        T = T * dh_matrix(DH_params(i,1), DH_params(i,2), DH_params(i,3), DH_params(i,4));
    end
end

% Jacobian matrisinin hesaplanması
function J = compute_jacobian(thetas)
    n = length(thetas);
    T = eye(4);
    % Ts dizisi, her eklemden sonraki dönüşüm matrislerini saklamak için
    Ts = cell(n+1, 1);
    Ts{1} = T;
    
    % D-H parametrelerinin tanımlanması
    DH_params = [0,      pi/2, 0.147, thetas(1);
                 0.155,  0,    0,     thetas(2);
                 0.135,  0,    0,     thetas(3);
                 0,      pi/2, 0.04,  thetas(4);
                 0,      0,    0.033,thetas(5)];
    
    for i = 1:n
        T = T * dh_matrix(DH_params(i,1), DH_params(i,2), DH_params(i,3), DH_params(i,4));
        Ts{i+1} = T;
    end
    
    % Son konum (end-effector) pozisyonu
    pe = T(1:3,4);
    J = zeros(6,n);
    for i = 1:n
        T_i = Ts{i};
        % Her eklem için z ekseni (rotasyon matrisi sütun 3)
        z = T_i(1:3,3);
        p = T_i(1:3,4);
        % Pozisyonel bileşen: z ekseni ile (pe - p) vektörünün dış çarpımı
        J_pos = cross(z, pe - p);
        % Rotasyonel bileşen: eklem ekseni
        J_rot = z;
        J(1:3,i) = J_pos;
        J(4:6,i) = J_rot;
    end
end

% İteratif İnvers Kinematik (IK) çözücüsü
function thetas = inverse_kinematics(target_pose, initial_thetas)
    max_iterations = 1000;
    threshold = 1e-4;
    alpha = 0.1;  % Güncelleme katsayısı (adım boyu)
    
    thetas = initial_thetas;
    target_p = target_pose(1:3,4);
    
    for iter = 1:max_iterations
        current_pose = forward_kinematics(thetas);
        current_p = current_pose(1:3,4);
        error = target_p - current_p;
        
        if norm(error) < threshold
            fprintf('%d iterasyonda yakınsama sağlandı.\n', iter);
            return;
        end
        
        % Jacobian hesaplanıyor ve sadece pozisyonel kısmı alınıyor
        J = compute_jacobian(thetas);
        J_pos = J(1:3,:);
        
        % Eklemlerde yapılacak güncelleme: Pseudo-inverse kullanılarak hesaplanır
        dtheta = alpha * pinv(J_pos) * error;
        thetas = thetas + dtheta;
    end
    
    fprintf('Maksimum iterasyon sayısına ulaşıldı, çözüm optimal olmayabilir.\n');
end

%%