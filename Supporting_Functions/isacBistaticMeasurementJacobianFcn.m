function H = isacBistaticMeasurementJacobianFcn(state, txPos, rxPos, rxOrientationAxes)
    % 计算双基地雷达测量函数的雅可比矩阵
    % 输入:
    %   state - 状态向量 [x; vx; y; vy] (CV) 或 [x; vx; y; vy; omega] (CT)
    %   txPos - 发射机位置 [x; y; z]
    %   rxPos - 接收机位置 [x; y; z]
    %   rxOrientationAxes - 接收机方向轴 (3x3矩阵)
    % 输出:
    %   H - 雅可比矩阵 (3 x length(state))
    
    % 提取状态变量
    x = state(1);
    vx = state(2);
    y = state(3);
    vy = state(4);
    
    n = length(state);  % 状态维度 (4 for CV, 5 for CT)
    H = zeros(3, n);    % 3个测量量
    
    % 计算中间变量
    dx_tx = x - txPos(1);
    dy_tx = y - txPos(2);
    dx_rx = x - rxPos(1);
    dy_rx = y - rxPos(2);
    
    rangeTx = sqrt(dx_tx^2 + dy_tx^2);
    rangeRx = sqrt(dx_rx^2 + dy_rx^2);
    
    % 防止除零
    if rangeTx < 1e-6
        rangeTx = 1e-6;
    end
    if rangeRx < 1e-6
        rangeRx = 1e-6;
    end
    
    % ========== 1. 双基地距离的雅可比 ==========
    % d(bistaticRange)/dx
    H(1, 1) = dx_tx/rangeTx + dx_rx/rangeRx;
    % d(bistaticRange)/dvx = 0
    H(1, 2) = 0;
    % d(bistaticRange)/dy
    H(1, 3) = dy_tx/rangeTx + dy_rx/rangeRx;
    % d(bistaticRange)/dvy = 0
    H(1, 4) = 0;
    
    % ========== 2. 到达角(AoA)的雅可比 ==========
    % 需要计算在接收机局部坐标系中的位置
    pos_rel_rx = [dx_rx; dy_rx; 0];
    pos_local = global2localcoord(pos_rel_rx, 'rs', [0;0;0], rxOrientationAxes.');
    
    xl = pos_local(1);
    yl = pos_local(2);
    
    % 在局部坐标系中的距离
    rho_local = sqrt(xl^2 + yl^2);
    
    if rho_local < 1e-6
        rho_local = 1e-6;
    end
    
    % AoA = atan2(yl, xl) (弧度), 转度数需乘以180/pi
    % d(atan2(yl,xl))/d(xl) = -yl/(xl^2+yl^2)
    % d(atan2(yl,xl))/d(yl) = xl/(xl^2+yl^2)
    
    daoa_dxl = -yl / (rho_local^2) * (180/pi);
    daoa_dyl = xl / (rho_local^2) * (180/pi);
    
    % 需要从局部坐标导数转换到全局坐标导数
    % pos_local = R^T * pos_global，其中 R = rxOrientationAxes^T
    % 所以 d(pos_local)/d(pos_global) = R^T
    R_inv = rxOrientationAxes.';  % 从全局到局部的旋转矩阵
    
    % d(aoa)/dx = d(aoa)/dxl * d(xl)/dx + d(aoa)/dyl * d(yl)/dx
    H(2, 1) = daoa_dxl * R_inv(1,1) + daoa_dyl * R_inv(2,1);
    H(2, 2) = 0;  % d(aoa)/dvx = 0
    H(2, 3) = daoa_dxl * R_inv(1,2) + daoa_dyl * R_inv(2,2);
    H(2, 4) = 0;  % d(aoa)/dvy = 0
    
    % ========== 3. 双基地距离速率的雅可比 ==========
    % bistaticRangeRate = v_tx + v_rx
    % v_tx = (vx*dx_tx + vy*dy_tx) / rangeTx
    % v_rx = (vx*dx_rx + vy*dy_rx) / rangeRx
    
    % d(v_tx)/dx
    dv_tx_dx = (vx/rangeTx) - (vx*dx_tx + vy*dy_tx)*dx_tx/(rangeTx^3);
    % d(v_tx)/dvx
    dv_tx_dvx = dx_tx / rangeTx;
    % d(v_tx)/dy
    dv_tx_dy = (vy/rangeTx) - (vx*dx_tx + vy*dy_tx)*dy_tx/(rangeTx^3);
    % d(v_tx)/dvy
    dv_tx_dvy = dy_tx / rangeTx;
    
    % d(v_rx)/dx
    dv_rx_dx = (vx/rangeRx) - (vx*dx_rx + vy*dy_rx)*dx_rx/(rangeRx^3);
    % d(v_rx)/dvx
    dv_rx_dvx = dx_rx / rangeRx;
    % d(v_rx)/dy
    dv_rx_dy = (vy/rangeRx) - (vx*dx_rx + vy*dy_rx)*dy_rx/(rangeRx^3);
    % d(v_rx)/dvy
    dv_rx_dvy = dy_rx / rangeRx;
    
    H(3, 1) = dv_tx_dx + dv_rx_dx;
    H(3, 2) = dv_tx_dvx + dv_rx_dvx;
    H(3, 3) = dv_tx_dy + dv_rx_dy;
    H(3, 4) = dv_tx_dvy + dv_rx_dvy;
    
    % 如果是CT模型 (5维状态)，omega对测量没有直接影响
    if n == 5
        H(1, 5) = 0;
        H(2, 5) = 0;
        H(3, 5) = 0;
    end
end
