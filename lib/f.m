function dy_dt = f(t, y)
    % 微分方程求解辅助函数: 调用各个函数计算导数
    % dy_dt = [dr_dt, dv_dt, dq_dt, dw_dt]
    % y = [r, v, q, w]
    global earth;
    global satellite;
    global sun;
    
    r = y(1:3);
    v = y(4:6);
    q = y(7:10);

    e = QuaternionToEuler(q);
    w = y(11:13);
    W = y(14:16);
    
    dt = satellite.Dt;
    i = floor(t/dt)+1;
    q_sun = sun.QList(i,:);
    
    % 生成制导指令
    [e_t, w_t, status] = satellite.Guidance(q_sun, r, e);
    
    % e_t = [0,0,0]; % 取消注释, 测试控制系统
    % w_t = [0,0,0]; % 取消注释, 测试控制系统

    satellite.TargetAttitudeList(i,:) = e_t;
    satellite.TargetAngularVelocityList(i,:) = w_t;
    satellite.StatusList(i,:) = status;
    
    % 生成控制指令
    % B:惯性坐标系下的地磁场强度
    B = earth.MagneticField(r);
    B = 1.0e-04 *[0.2178, 0.0020, 0]; % 取消注释, 定值磁场强度
    [dW_dt, M_m] = satellite.Control(e, w, e_t, w_t, W, B);
    satellite.MagneticTorqueList(i,:) = M_m;

    % 轨道动力学
    [dr_dt, dv_dt] = satellite.OrbitDynamics(r,v,earth);

    % 姿态动力学
    M_d = [0,0,0]; % 干扰力矩
    
    M_m = [0,0,0];% 取消注释, 关闭磁力矩器
    
    [dq_dt, dw_dt] = satellite.AttitudeDynamics(q, w, dW_dt, W, M_m, M_d);
    
    dy_dt = [dr_dt, dv_dt, dq_dt, dw_dt, dW_dt];
end