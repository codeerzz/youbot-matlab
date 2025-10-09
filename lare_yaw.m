%% Parametreler
Ts = 0.001;       % Örnekleme zamanı (1 ms)
t = 0:Ts:30;      % 30 saniyelik simülasyon süresi
N = length(t);

% Her kenar 7.5 saniye sürecek
T_seg = 7.5;
Uz = 2 
v = Uz / T_seg;    % Her kenarda 5 metre ilerleyecek, sabit hız (0.6667 m/s)


% Ön tanımlamalar
xRef = zeros(1, N);
yRef = zeros(1, N);
dxRef = zeros(1, N);
dyRef = zeros(1, N);
thetaRaw = zeros(1, N);  % Yön açısı

for i = 1:N
    ti = t(i);
    
    if ti <= T_seg  % 1. segment: +x
        xRef(i) = v * ti;
        yRef(i) = 0;
        dxRef(i) = v;
        dyRef(i) = 0;
        thetaRaw(i) = 0;

    elseif ti <= 2*T_seg  % 2. segment: +y
        xRef(i) = Uz;
        yRef(i) = v * (ti - T_seg);
        dxRef(i) = 0;
        dyRef(i) = v;
        thetaRaw(i) = pi/2;

    elseif ti <= 3*T_seg  % 3. segment: -x
        xRef(i) = Uz - v * (ti - 2*T_seg);
        yRef(i) = Uz;
        dxRef(i) = -v;
        dyRef(i) = 0;
        thetaRaw(i) = pi;

    else  % 4. segment: -y
        xRef(i) = 0;
        yRef(i) = Uz - v * (ti - 3*T_seg);
        dxRef(i) = 0;
        dyRef(i) = -v;
        thetaRaw(i) = -pi/2;
    end
end

% İvme sıfır çünkü sabit hız
ddxRef = zeros(1, N);
ddyRef = zeros(1, N);

% Hız ve ivme filtrelemesi ve limitleri
dxRef = smoothdata(dxRef, 'gaussian', 15);
dyRef = smoothdata(dyRef, 'gaussian', 15);
V_MAX = 1.2;
V_MIN = -1.2;
dxRef = max(min(dxRef, V_MAX), V_MIN);
dyRef = max(min(dyRef, V_MAX), V_MIN);

ddxRef = smoothdata(ddxRef, 'gaussian', 15);
ddyRef = smoothdata(ddyRef, 'gaussian', 15);
A_MAX = 2.0;
A_MIN = -2.0;
ddxRef = max(min(ddxRef, A_MAX), A_MIN);
ddyRef = max(min(ddyRef, A_MAX), A_MIN);

% Yön açısı ve türevleri
thetaRef = unwrap(thetaRaw);  % Ani geçişlerde düzgün ilerlesin
dthetaRef = [0, diff(thetaRef)/Ts];
ddthetaRef = [0, diff(dthetaRef)/Ts];
dthetaRef = smoothdata(dthetaRef, 'gaussian', 15);
ddthetaRef = smoothdata(ddthetaRef, 'gaussian', 15);
W_MAX = 0.8;
W_MIN = -0.8;
dthetaRef = max(min(dthetaRef, W_MAX), W_MIN);
ALPHA_MAX = 1.5;
ALPHA_MIN = -1.5;
ddthetaRef = max(min(ddthetaRef, ALPHA_MAX), ALPHA_MIN);

% Simulink formatına çevir
qRef_simulink  = timeseries([xRef'  yRef'  thetaRef'],  t');
dqRef_simulink = timeseries([dxRef' dyRef' dthetaRef'], t');
ddqRef_simulink= timeseries([ddxRef' ddyRef' ddthetaRef'], t');

% Kaydet
save('reference_trajectory_square.mat', ...
     'qRef_simulink', 'dqRef_simulink', 'ddqRef_simulink');

disp('Yöne göre dönen robot için kare referans trajektorisi kaydedildi.');
