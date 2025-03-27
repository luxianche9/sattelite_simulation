% 测试
% Class Satellite
% method OrbitDynamics
clear;
clc;

% 仿真时间设置
start_date = "2024-12-15-06-31-00";
time = 60*60*24;% s
dt = 10;% s
length = floor(time/dt);

global earth;
global satellite;
earth = Earth(start_date, time, dt);
satellite = Satellite(time, dt);

% 卫星初始位置, 速度
a = 9000; % 半长轴
e = 0.001; % 偏心率
i = deg2rad(42.1); % 轨道倾角
Omega = deg2rad(20); % 升交点赤经
omega = deg2rad(0); % 近地点辐角
theta = deg2rad(0); % 真近点角
[r0, v0] = ElementsToRV(earth.Mu,a,e,i,Omega,omega,theta);

% 数值求解轨道
[r_list, v_list] = simulation(r0, v0, time, dt);
satellite.PositionList = r_list;
satellite.VelocityList = v_list;

% 可视化
figure;
title('可视化卫星轨道');
hold on;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
rotate3d on;
for n = 1:length - 1
    time = n * dt;
    cla;

    DrawCube(8);
    earth.DrawEarth(time, 2);
    satellite.DrawOrbit(time, 2);

    pause(0.000001);
end

function [r_list, v_list] = simulation(r0, v0, time, dt)
    % 微分方程求解辅助函数: 处理多变量
    y0 = [r0,v0];
    y = EulerPredictCorrect(time, dt, y0, @f);
    r_list = y(:,1:3);
    v_list = y(:,4:6);
end

function dy_dt = f(t, y)
    % 微分方程求解辅助函数: 调用各个函数计算导数
    % dy_dt = [dr_dt, dv_dt]
    % y = [r, v]

    global earth;
    global satellite;

    r = y(1:3);
    v = y(4:6);

    % 轨道动力学
    [dr_dt, dv_dt] = satellite.OrbitDynamics(r,v,earth);

    dy_dt = [dr_dt , dv_dt];
end