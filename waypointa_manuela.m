

% Tüm waypoint'lerin birikeceği ana matris. (Nx2 formatında olacak)
% İlk sütun X, ikinci sütun Y koordinatlarını tutacak.
waypoints = [];

% Başlangıç konumu [X, Y] olarak belirlenir.
mevcut_konum = [0, 0];

disp('--- Basit Waypoint Oluşturucu ---');
disp('Her adımda hedef X ve Y koordinatını girin.');
disp('Çıkmak için koordinat girmeden doğrudan Enter''a basın.');
disp('----------------------------------------------------');

% Kullanıcıdan sürekli olarak yeni hedef alan ana döngü
while true
    
    fprintf('\nMevcut Konum: [X: %.2f, Y: %.2f]\n', mevcut_konum(1), mevcut_konum(2));
    
    % Kullanıcıdan bir sonraki hedef X koordinatını al
    hedef_x_str = input('Bir sonraki hedef X koordinatını girin: ', 's');
    
    % Eğer kullanıcı boş bir değer girerse (sadece Enter'a basarsa) döngüyü sonlandır
    if isempty(hedef_x_str)
        disp('Giriş yapılmadı, program sonlandırılıyor.');
        break;
    end
    
    % Kullanıcıdan bir sonraki hedef Y koordinatını al
    hedef_y_str = input('Bir sonraki hedef Y koordinatını girin: ', 's');
    
    if isempty(hedef_y_str)
        disp('Giriş yapılmadı, program sonlandırılıyor.');
        break;
    end
    
    % Alınan string (metin) girdileri sayısal (double) değere dönüştür
    hedef_x = str2double(hedef_x_str);
    hedef_y = str2double(hedef_y_str);
    
    % Girdinin geçerli bir sayı olup olmadığını kontrol et
    if isnan(hedef_x) || isnan(hedef_y)
        disp('Hatalı giriş. Lütfen sayısal bir değer girin.');
        continue; % Döngünün başına dön
    end
    
    % Hedef konumu bir vektörde birleştir
    hedef_konum = [hedef_x, hedef_y];
    
    % Mevcut konum ile hedef konum arasını 10 eşit parçaya böl.
    % Bu işlem 1 başlangıç + 10 ara nokta = toplam 11 nokta oluşturur.
    % linspace(baslangic, bitis, nokta_sayisi)
    ara_x_noktalari = linspace(mevcut_konum(1), hedef_konum(1), 11);
    ara_y_noktalari = linspace(mevcut_konum(2), hedef_konum(2), 11);
    
    % Yeni oluşturulan noktaları geçici bir matrise ata.
    % Başlangıç noktasını (ilk elemanı) atlıyoruz çünkü o zaten bir önceki hedefti.
    % Bu sayede listemizde tekrar eden nokta olmaz.
    yeni_noktalar = [ara_x_noktalari(2:end)', ara_y_noktalari(2:end)'];
    
    % Yeni noktaları ana "waypoints" matrisine alt alta ekle
    waypoints = [waypoints; yeni_noktalar];
    
    % Mevcut konumu, bir sonraki döngü için son hedef olarak güncelle
    mevcut_konum = hedef_konum;
    
    % Her adımdan sonra güncel waypoint listesini ekrana yazdır
    fprintf('--- Güncel Waypoint Listesi (%d nokta) ---\n', size(waypoints, 1));
    disp(waypoints);
    
end

disp('--- Final Waypoint Matrisi ---');
disp('Workspace''te "waypoints" değişkeni olarak kaydedildi.');
disp(waypoints);