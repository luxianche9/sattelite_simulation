% 测试
% Class Satellite
% method DetectedSun
clear;
clc;

% 仿真时间设置
start_date = "2024-12-24-13-30-00";
time = 8.4972e+03;%(s) 2*pi*sqrt(a^3/earth.Mu)
dt = 1;%(s)
length = floor(time/dt);

global earth;
global satellite;
global sun;
earth = Earth(start_date, time, dt);
satellite = Satellite(time, dt);
sun = Sun(start_date, dt, time);

% 卫星初始位置, 速度
a = 9000; % 半长轴
e = 0; % 偏心率
i = deg2rad(0); % 轨道倾角
Omega = deg2rad(20); % 升交点赤经
omega = deg2rad(0); % 近地点辐角
theta = deg2rad(0); % 真近点角
% r0: 初始位置[x,y,z](km)
% v0: 初始速度[v_x, v_y, v_z](km/s)
[r0, v0] = ElementsToRV(earth.Mu,a,e,i,Omega,omega,theta);
% e0: 初始姿态[roll, pitch, yaw](rad)
% w0: 初始角速度[w_x, w_y, w_z](rad/s)
e0 = [0, 0, 0];
w0 = [0.1, 0, 0];

% 数值求解轨道, 姿态
[r_list, v_list, e_list, w_list] = simulation(r0, v0, e0, w0, time, dt);
satellite.PositionList = r_list;
satellite.VelocityList = v_list;
satellite.AttitudeList = e_list;
satellite.AngularVelocityList = w_list;


figure;
title('可视化卫星轨道');
hold on;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
rotate3d on;
time = 0;
attitude = [0,-pi/3*0.99,0];
q = [0,1,0];

satellite.AttitudeList(1,:) = attitude;
[detect_sun, detect_by_sensor_x, detect_by_sensor_z]...
                = satellite.DetectedSun(attitude, q);

status = [0,0,0,detect_by_sensor_x, detect_by_sensor_z];
satellite.DrawCubeSat(0, 0, 1, status);
quiver3(0,0,0,q(1)*1,q(2)*1,q(3)*1);


function [r_list, v_list, e_list, w_list]...
       = simulation(r0, v0, e0, w0, time, dt)
    % 微分方程求解辅助函数: 处理多变量
    % 欧拉角转换为四元数
    % r 位置[x,y,z](km)
    % v 速度[v_x, v_y, v_z](km/s)
    % e 姿态[roll, pitch, yaw](rad)
    % q 四元数[w, x, y, z]
    % w 角速度[w_x, w_y, w_z](rad/s)
    % s 传感器数据[in_shade, detect_sun, spin]

    q0 = EulerToQuaternion(e0);
    y0 = [r0,v0,q0,w0];
    
    % 欧拉矫正预测法求解微分方程
    y = EulerPredictCorrect(time, dt, y0, @f);
    
    r_list = y(:,1:3);
    v_list = y(:,4:6);

    % 四元数转换为欧拉角
    q_list = y(:,7:10);
    length_of_list = size(q_list,1);
    e_list = zeros(length_of_list, 3);
    for i = 1: length_of_list
        q = QuatNormalize(q_list(i,:));
        e_list(i,:) = QuaternionToEuler(q);
    end

    w_list = y(:,11:13);
end

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
    dt = satellite.Dt;
    i = floor(t/dt)+1;
    q_sun = sun.QList(i,:);
    
    % 制导
    [e_t, w_t, status] = satellite.Guidance(q_sun, r, e);
    satellite.TargetAttitudeList(i,:) = e_t;
    satellite.TargetAngularVelocityList(i,:) = w_t;
    satellite.StatusList(i,:) = status;

    % 轨道动力学
    [dr_dt, dv_dt] = satellite.OrbitDynamics(r,v,earth);
    % 姿态动力学
    [dq_dt, dw_dt] = satellite.AttitudeDynamics(q, w);
    
    dy_dt = [dr_dt, dv_dt, dq_dt, dw_dt];
end