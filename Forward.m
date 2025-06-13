%% Parametreler
Ts = 0.001;       % Örnekleme zamanı (1 ms)
t = 0:Ts:30;      % 30 saniyelik simülasyon süresi
N = length(t);

%% 5 metre x ekseninde düz çizgi için referans tanımı
% 30 saniyede 0'dan 5 metreye ulaşmak isteyen sabit bir hız profili:
xRef = (5/30) * t;     % (5 metre / 30 saniye) * t
yRef = zeros(size(t)); % y ekseni sabit (0)

%% Hız Referansları (1. türev)
% İstenilen sabit hız: 5/30 m/s (yaklaşık 0.1667 m/s)
dxRef = (5/30) * ones(1, N);
dyRef = zeros(1, N);

%% İvme Referansları (2. türev)
% Sabit hızda ivme (2. türev) sıfır
ddxRef = zeros(1, N);
ddyRef = zeros(1, N);

%% Hızları (opsiyonel) filtreleme ve limit (pek gerek kalmasa da önceki yapıyı koruyoruz)
% Gaussian filtre (sabit hızda etkisi olmaz, ama yapıyı koruyoruz)
dxRef = smoothdata(dxRef, 'gaussian', 15);
dyRef = smoothdata(dyRef, 'gaussian', 15);

% Hız limitleme
V_MAX = 1.2;   % m/s
V_MIN = -1.2;
dxRef = max(min(dxRef, V_MAX), V_MIN);
dyRef = max(min(dyRef, V_MAX), V_MIN);

%% İvme değerleri (opsiyonel) filtreleme ve limit (zaten sıfır ama yine de yapı korunsun)
ddxRef = smoothdata(ddxRef, 'gaussian', 15);
ddyRef = smoothdata(ddyRef, 'gaussian', 15);

A_MAX = 2.0;   % m/s²
A_MIN = -2.0;
ddxRef = max(min(ddxRef, A_MAX), A_MIN);
ddyRef = max(min(ddyRef, A_MAX), A_MIN);

%% Theta (Yön Açısı) Hesaplama
% Pozitif x ekseninde hareket -> açı 0
% Yine de atan2 kullanabiliriz (dy=0, dx>0) -> 0
thetaRaw = atan2(dyRef, dxRef);
thetaRef = unwrap(thetaRaw);

%% Açısal Hız ve İvme Hesaplama
% Sabit doğrultuda ilerleme -> açısal hız & ivme sıfır
dthetaRef = zeros(1, N);
ddthetaRef = zeros(1, N);

% (Önceki kod yapısını korumak istersek smoothdata veya limit uygulayabiliriz)
dthetaRef = smoothdata(dthetaRef, 'gaussian', 15);
ddthetaRef = smoothdata(ddthetaRef, 'gaussian', 15);

W_MAX = 0.8;   % rad/s
W_MIN = -0.8;
dthetaRef = max(min(dthetaRef, W_MAX), W_MIN);

ALPHA_MAX = 1.5;  % rad/s²
ALPHA_MIN = -1.5;
ddthetaRef = max(min(ddthetaRef, ALPHA_MAX), ALPHA_MIN);

%% Simulink İçin 3x1 Formatında Referans Verisi
qRef_simulink  = timeseries([xRef'  yRef'  thetaRef'],  t');
dqRef_simulink = timeseries([dxRef' dyRef' dthetaRef'], t');
ddqRef_simulink= timeseries([ddxRef' ddyRef' ddthetaRef'], t');

%% Kaydetme
save('reference_trajectory.mat', ...
     'qRef_simulink', 'dqRef_simulink', 'ddqRef_simulink');

disp('5 metre x ekseninde sabit hız (5/30 m/s), ivmesi sıfır olan referans kaydedildi.');
