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
    cvStateTransitionJacobianFcn = @(state, dt_input) cvJacobian(state, dt_input);
    
    % 创建CV滤波器
    cvFilter = trackingEKF(...
        'State', cvState, ...
        'StateCovariance', diag([10^2 5^2 10^2 5^2]), ...
        'StateTransitionFcn', cvStateTransitionFcn, ...
        'StateTransitionJacobianFcn', cvStateTransitionJacobianFcn, ...
        'ProcessNoise', Q_cv, ...
        'MeasurementFcn', @(state) isacBistaticMeasurementFcn(state, txPos, rxPos, rxOrientationAxes), ...
    'MeasurementJacobianFcn', @(state) isacBistaticMeasurementJacobianFcn(state, txPos, rxPos, rxOrientationAxes), ...
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
    ctStateTransitionJacobianFcn = @(state, dt_input) coordinatedTurnJacobian(state, dt_input);
    
    % 创建CT滤波器
    ctFilter = trackingEKF(...
        'State', ctState, ...
        'StateCovariance', P_ct, ...
        'StateTransitionFcn', ctStateTransitionFcn, ...
        'StateTransitionJacobianFcn', ctStateTransitionJacobianFcn, ...
        'ProcessNoise', Q_ct, ...
        'MeasurementFcn', @(state) isacBistaticMeasurementFcn(state, txPos, rxPos, rxOrientationAxes), ...
    'MeasurementJacobianFcn', @(state) isacBistaticMeasurementJacobianFcn(state, txPos, rxPos, rxOrientationAxes), ...
    'MeasurementNoise', R);
    
    % === 创建IMM滤波器 ===
    % 模型转移概率矩阵
    transitionProb = [0.95 0.05;   % CV -> CV: 0.95, CV -> CT: 0.05
                      0.05 0.95];  % CT -> CV: 0.05, CT -> CT: 0.95
    
    % 初始模型概率
    initialModelProb = [0.5; 0.5];  % 两个模型初始概率相等
    
    % 创建IMM，使用ModelNames指定模型类型
    filter = trackingIMM({cvFilter, ctFilter}, ...
        'ModelNames', {'constvel', 'constturn'}, ...
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

function J = cvJacobian(~, dt)
    % CV模型雅可比矩阵
    % 第一个参数是state（未使用，用~表示）
    J = [1 dt 0 0;
         0 1  0 0;
         0 0  1 dt;
         0 0  0 1];
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

function J = coordinatedTurnJacobian(state, dt)
    % CT模型的雅可比矩阵
    vx = state(2);
    vy = state(4);
    omega = state(5);
    
    if abs(omega) < 1e-6
        % omega接近0时，使用CV的雅可比
        J = [1 dt 0 0  0;
             0 1  0 0  0;
             0 0  1 dt 0;
             0 0  0 1  0;
             0 0  0 0  1];
    else
        sinw = sin(omega * dt);
        cosw = cos(omega * dt);
        
        % 计算偏导数
        J = zeros(5, 5);
        J(1,1) = 1;
        J(1,2) = sinw/omega;
        J(1,3) = 0;
        J(1,4) = (cosw-1)/omega;
        J(1,5) = (vx*(omega*dt*cosw - sinw) + vy*(-omega*dt*sinw - cosw + 1))/(omega^2);
        
        J(2,1) = 0;
        J(2,2) = cosw;
        J(2,3) = 0;
        J(2,4) = -sinw;
        J(2,5) = -dt*(vx*sinw + vy*cosw);
        
        J(3,1) = 0;
        J(3,2) = (1-cosw)/omega;
        J(3,3) = 1;
        J(3,4) = sinw/omega;
        J(3,5) = (vx*(-omega*dt*sinw - cosw + 1) + vy*(omega*dt*cosw - sinw))/(omega^2);
        
        J(4,1) = 0;
        J(4,2) = sinw;
        J(4,3) = 0;
        J(4,4) = cosw;
        J(4,5) = dt*(vx*cosw - vy*sinw);
        
        J(5,1) = 0;
        J(5,2) = 0;
        J(5,3) = 0;
        J(5,4) = 0;
        J(5,5) = 1;
    end
end


