function z = isacBistaticMeasurementFcn(state, txPos, rxPos, rxOrientationAxes)
    % 双基地雷达的非线性测量函数
    % 输入:
    %   state - 状态向量 [x; vx; y; vy] (CV) 或 [x; vx; y; vy; omega] (CT) 或 [x; vx; ax; y; vy; ay] (CA)
    %   txPos - 发射机位置 [x; y; z]
    %   rxPos - 接收机位置 [x; y; z]
    %   rxOrientationAxes - 接收机方向轴 (3x3矩阵)
    % 输出:
    %   z - 测量向量 [bistaticRange; aoa; bistaticRangeRate]
    
    % 提取状态变量（位置和速度始终在前4个分量中）
    x = state(1);
    vx = state(2);
    if length(state) == 6  % CA模型：[x; vx; ax; y; vy; ay]
        y = state(4);
        vy = state(5);
    else  % CV或CT模型：[x; vx; y; vy; ...]
        y = state(3);
        vy = state(4);
    end
    
    % 1. 计算双基地距离 (Bistatic Range)
    % 从发射机到目标的距离
    rangeTx = sqrt((x - txPos(1))^2 + (y - txPos(2))^2);
    % 从目标到接收机的距离
    rangeRx = sqrt((x - rxPos(1))^2 + (y - rxPos(2))^2);
    % 双基地距离（减去基线以匹配原始实现）
    baseline = vecnorm(txPos - rxPos);
    bistaticRange = rangeTx + rangeRx - baseline;
    
    % 2. 计算到达角 (AoA)
    % 目标相对于接收机的位置
    pos_rel_rx = [x - rxPos(1); y - rxPos(2); 0];
    % 转换到接收机局部坐标系
    pos_local = global2localcoord(pos_rel_rx, 'rs', [0;0;0], rxOrientationAxes.');
    % 计算方位角
    [aoa, ~, ~] = cart2sph(pos_local(1), pos_local(2), pos_local(3));
    aoa = rad2deg(aoa); % 转换为度
    
    % 3. 计算双基地距离速率 (Bistatic Range Rate)
    % 目标朝向发射机的径向速度分量
    if rangeTx > 0
        v_tx = (vx*(x-txPos(1)) + vy*(y-txPos(2))) / rangeTx;
    else
        v_tx = 0;
    end
    
    % 目标朝向接收机的径向速度分量
    if rangeRx > 0
        v_rx = (vx*(x-rxPos(1)) + vy*(y-rxPos(2))) / rangeRx;
    else
        v_rx = 0;
    end
    
    % 双基地距离速率是两个径向速度之和
    bistaticRangeRate = v_tx + v_rx;
    
    % 组装测量向量
    z = [bistaticRange; aoa; bistaticRangeRate];
end
