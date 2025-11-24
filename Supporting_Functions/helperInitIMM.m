function filter = helperInitIMM(detection, txPos, rxPos, txOrientationAxes, rxOrientationAxes)
    % 为IMM跟踪器初始化滤波器
    % 输入:
    %   detection - objectDetection对象
    %   txPos - 发射机位置
    %   rxPos - 接收机位置
    %   txOrientationAxes - 发射机方向轴
    %   rxOrientationAxes - 接收机方向轴
    
    % 从检测中估计初始状态（笛卡尔坐标）
    % 使用双基地位置估计
    meassph = zeros(3, 1);
    meassph(1) = detection.Measurement(2);  % azimuth
    meassph(3) = detection.Measurement(1);  % range
    meassph = local2globalcoord(meassph, 'ss', [0; 0; 0], rxOrientationAxes);
    
    meascart = bistaticposest(meassph(3), meassph(1:2), eps, [eps; eps], ...
        txPos, rxPos, 'RangeMeasurement', 'BistaticRange');
    
    % 初始位置
    x0 = meascart(1);
    y0 = meascart(2);
    
    % 初始速度估计
    if length(detection.Measurement) >= 3
        rangeRate = detection.Measurement(3);
        % 简单估计：假设径向速度主要沿着接收机方向
        dx_rx = x0 - rxPos(1);
        dy_rx = y0 - rxPos(2);
        rangeRx = sqrt(dx_rx^2 + dy_rx^2);
        if rangeRx > 0
            vx0 = rangeRate * dx_rx / rangeRx / 2;  % 除以2因为是双基地
            vy0 = rangeRate * dy_rx / rangeRx / 2;
        else
            vx0 = 0;
            vy0 = 0;
        end
    else
        vx0 = 0;
        vy0 = 0;
    end
    
    % === 定义CV滤波器（4维状态：[x; vx; y; vy]） ===
    cvState = [x0; vx0; y0; vy0];
    
    % 过程噪声协方差
    dt = 0.4;  % 时间步长
    q = 5;  % 过程噪声强度
    Q_cv = q^2 * [dt^4/4 dt^3/2 0      0;
                  dt^3/2 dt^2   0      0;
                  0      0      dt^4/4 dt^3/2;
                  0      0      dt^3/2 dt^2];
    
    % 测量噪声协方差（3个测量：range, azimuth, rangeRate）
    R = diag([detection.MeasurementNoise(1,1), ...
              detection.MeasurementNoise(2,2), ...
              0.5^2]);  % 径向速度噪声
    
    % 注意：状态转移函数必须接受dt作为第二个参数
    cvStateTransitionFcn = @(state, dt_input) cvTransition(state, dt_input);
    
    % 创建CV滤波器（使用UKF，不需要雅可比函数）
    cvFilter = trackingUKF(...
        'State', cvState, ...
        'StateCovariance', diag([10^2 5^2 10^2 5^2]), ...
        'StateTransitionFcn', cvStateTransitionFcn, ...
        'ProcessNoise', Q_cv, ...
        'MeasurementFcn', @(state) isacBistaticMeasurementFcn(state, txPos, rxPos, rxOrientationAxes), ...
        'MeasurementNoise', R);
    
    % === 定义CT滤波器（5维状态：[x; vx; y; vy; omega]） ===
    omega0 = 0;  % 初始角速度
    ctState = [x0; vx0; y0; vy0; omega0];
    
    % 初始协方差
    P_ct = diag([10^2 5^2 10^2 5^2 (pi/10)^2]);  % omega的不确定性
    
    % CT过程噪声（5维）
    Q_ct = blkdiag(Q_cv, 0.1^2);
    
    % CT状态转移函数（接受dt参数）
    ctStateTransitionFcn = @(state, dt_input) coordinatedTurnTransition(state, dt_input);
    
    % 创建CT滤波器（使用UKF，不需要雅可比函数）
    ctFilter = trackingUKF(...
        'State', ctState, ...
        'StateCovariance', P_ct, ...
        'StateTransitionFcn', ctStateTransitionFcn, ...
        'ProcessNoise', Q_ct, ...
        'MeasurementFcn', @(state) isacBistaticMeasurementFcn(state, txPos, rxPos, rxOrientationAxes), ...
        'MeasurementNoise', R);
    
    % === 定义CA滤波器（6维状态：[x; vx; ax; y; vy; ay]） ===
    ax0 = 0;  % 初始x方向加速度
    ay0 = 0;  % 初始y方向加速度
    caState = [x0; vx0; ax0; y0; vy0; ay0];
    
    % 初始协方差（位置、速度、加速度）
    P_ca = diag([10^2 5^2 2^2 10^2 5^2 2^2]);  % 加速度不确定性为2 m/s²
    
    % CA过程噪声（6维）- 假设加加速度（jerk）为白噪声
    q_jerk = 1;  % 加加速度噪声强度 (m/s³)
    Q_ca = q_jerk^2 * [dt^5/20  dt^4/8   dt^3/6   0        0        0;
                       dt^4/8   dt^3/3   dt^2/2   0        0        0;
                       dt^3/6   dt^2/2   dt       0        0        0;
                       0        0        0        dt^5/20  dt^4/8   dt^3/6;
                       0        0        0        dt^4/8   dt^3/3   dt^2/2;
                       0        0        0        dt^3/6   dt^2/2   dt];
    
    % CA状态转移函数（接受dt参数）
    caStateTransitionFcn = @(state, dt_input) caTransition(state, dt_input);
    
    % 创建CA滤波器（使用UKF，不需要雅可比函数）
    caFilter = trackingUKF(...
        'State', caState, ...
        'StateCovariance', P_ca, ...
        'StateTransitionFcn', caStateTransitionFcn, ...
        'ProcessNoise', Q_ca, ...
        'MeasurementFcn', @(state) isacBistaticMeasurementFcn(state, txPos, rxPos, rxOrientationAxes), ...
        'MeasurementNoise', R);
    
    % === 创建IMM滤波器（3模型）===
    % 模型转移概率矩阵（3x3）
    transitionProb = [0.90 0.05 0.05;   % CV -> CV: 0.90, CV -> CT: 0.05, CV -> CA: 0.05
                      0.05 0.90 0.05;   % CT -> CV: 0.05, CT -> CT: 0.90, CT -> CA: 0.05
                      0.05 0.05 0.90];  % CA -> CV: 0.05, CA -> CT: 0.05, CA -> CA: 0.90
    
    % 初始模型概率（3个模型）
    initialModelProb = [0.6; 0.2; 0.2];  % CV优先，CT和CA次之
    
    % 创建IMM，使用ModelNames指定模型类型
    % 注意：MATLAB支持的模型名称为 'constvel', 'constturn', 'constacc'
    filter = trackingIMM({cvFilter, ctFilter, caFilter}, ...
        'ModelNames', {'constvel', 'constturn', 'constacc'}, ...
        'TransitionProbabilities', transitionProb, ...
        'ModelProbabilities', initialModelProb);
end

function newState = cvTransition(state, dt)
    % CV模型状态转移
    F = [1 dt 0 0;
         0 1  0 0;
         0 0  1 dt;
         0 0  0 1];
    newState = F * state;
end

function newState = coordinatedTurnTransition(state, dt)
    % 恒定转弯率（CT）模型的状态转移函数
    x = state(1);
    vx = state(2);
    y = state(3);
    vy = state(4);
    omega = state(5);
    
    if abs(omega) < 1e-6
        % 当omega接近0时，退化为CV模型
        newState = [x + vx*dt; vx; y + vy*dt; vy; omega];
    else
        % CT模型
        sinw = sin(omega * dt);
        cosw = cos(omega * dt);
        
        newState = [
            x + (vx*sinw + vy*(cosw-1))/omega;
            vx*cosw - vy*sinw;
            y + (vx*(1-cosw) + vy*sinw)/omega;
            vx*sinw + vy*cosw;
            omega
        ];
    end
end

function newState = caTransition(state, dt)
    % 恒定加速度（CA）模型的状态转移函数
    % 状态向量：[x; vx; ax; y; vy; ay]
    x = state(1);
    vx = state(2);
    ax = state(3);
    y = state(4);
    vy = state(5);
    ay = state(6);
    
    % CA模型状态转移方程
    % x_{k+1} = x_k + vx_k*dt + 0.5*ax_k*dt^2
    % vx_{k+1} = vx_k + ax_k*dt
    % ax_{k+1} = ax_k (恒定加速度)
    % y_{k+1} = y_k + vy_k*dt + 0.5*ay_k*dt^2
    % vy_{k+1} = vy_k + ay_k*dt
    % ay_{k+1} = ay_k (恒定加速度)
    
    newState = [
        x + vx*dt + 0.5*ax*dt^2;
        vx + ax*dt;
        ax;
        y + vy*dt + 0.5*ay*dt^2;
        vy + ay*dt;
        ay
    ];
end
