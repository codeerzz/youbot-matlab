%% Parametreler%% Parametreler
Ts = 0.001;     % Yeni örnekleme zamanı (1 ms)
t = 0:Ts:30;    % 30 saniyelik simülasyon süresi

%% Referans Yörünge
freq = 2*pi/30;
xRef = 1.1 + 0.7*sin(freq*t);  
yRef = 1.1 + 0.7*sin(2*freq*t);

%% Ham Hız Referansları
dxRef = freq * 0.7 * cos(freq*t);   
dyRef = 2 * freq * 0.7 * cos(2*freq*t);

%% Hızları Gaussian Filtre ile Yumuşatma
dxRef = smoothdata(dxRef, 'gaussian', 15);
dyRef = smoothdata(dyRef, 'gaussian', 15);

%% Hız Limitleme
V_MAX = 1.2; % m/s
V_MIN = -1.2;
dxRef = max(min(dxRef, V_MAX), V_MIN);
dyRef = max(min(dyRef, V_MAX), V_MIN);

%% Ham İvme Referansları
ddxRef = diff(dxRef) / Ts; 
ddyRef = diff(dyRef) / Ts;

% Boyutları eşitle (diff çıkışı 1 eleman azaldı)
ddxRef = [ddxRef, ddxRef(end)];
ddyRef = [ddyRef, ddyRef(end)];

%% İvme Değerlerini Yumuşatma
ddxRef = smoothdata(ddxRef, 'gaussian', 15);
ddyRef = smoothdata(ddyRef, 'gaussian', 15);

%% İvme Limitleme
A_MAX = 2.0;  % m/s²
A_MIN = -2.0;
ddxRef = max(min(ddxRef, A_MAX), A_MIN);
ddyRef = max(min(ddyRef, A_MAX), A_MIN);

%% Theta (Yön Açısı) Hesaplama ve Unwrap
% 1) İlk olarak atan2 ile açıları bul
thetaRaw = atan2(dyRef, dxRef);
% 2) Wrap sorununu önlemek için unwrap kullan
thetaRef = unwrap(thetaRaw);

%% Açısal Hız ve İvme Hesaplama
dthetaRef = diff(thetaRef) / Ts; 
dthetaRef = [dthetaRef, dthetaRef(end)];  % Boyut eşitleme
ddthetaRef = diff(dthetaRef) / Ts; 
ddthetaRef = [ddthetaRef, ddthetaRef(end)];

%% Açısal Hız ve İvme için Gaussian Filtre
dthetaRef = smoothdata(dthetaRef, 'gaussian', 15);
ddthetaRef = smoothdata(ddthetaRef, 'gaussian', 15);

%% Açısal Hız ve İvme Limitleri
W_MAX = 0.8;  % rad/s
W_MIN = -0.8;
dthetaRef = max(min(dthetaRef, W_MAX), W_MIN);

ALPHA_MAX = 1.5;  % rad/s²
ALPHA_MIN = -1.5;
ddthetaRef = max(min(ddthetaRef, ALPHA_MAX), ALPHA_MIN);

%% Simulink İçin 3x1 Formatında Referans Verisi
qRef_simulink = timeseries([xRef' yRef' thetaRef'], t');
dqRef_simulink = timeseries([dxRef' dyRef' dthetaRef'], t');
ddqRef_simulink = timeseries([ddxRef' ddyRef' ddthetaRef'], t');

%% Kaydetme
save('reference_trajectory.mat', ...
     'qRef_simulink','dqRef_simulink','ddqRef_simulink');
disp('YouBot base için "unwrap" ile düzeltilmiş açısal referans verileri kaydedildi.');
