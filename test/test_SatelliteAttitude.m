% 测试
% Class Satellite
% method AttitudeDynamic
clear;
clc;

% 仿真时间设置
start_date = "2024-12-15-06-31-00";
time = 10;% s
dt = 0.1;% s
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
% r0: 初始位置[x,y,z](km)
% v0: 初始速度[v_x, v_y, v_z](km/s)
[r0, v0] = ElementsToRV(earth.Mu,a,e,i,Omega,omega,theta);
% e0: 初始姿态[roll, pitch, yaw](rad)
% w0: 初始角速度[w_x, w_y, w_z](rad/s)
e0 = [0, 0, 0];
w0 = [0, 0, 2*pi/time];

% 数值求解轨道, 姿态
[r_list, v_list, e_list, w_list] = simulation(r0, v0, e0, w0, time, dt);
satellite.PositionList = r_list;
satellite.VelocityList = v_list;
satellite.AttitudeList = e_list;
satellite.AngularVelocityList = w_list;

% 可视化
videoFileName = 'satellite_orbit_animation.avi'; % 视频文件名
% 视频对象
video = VideoWriter(videoFileName, 'Motion JPEG AVI'); % 创建视频写入对象
video.FrameRate = 1/dt; % 设置帧率
open(video); % 打开视频文件

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

    DrawCube(3);
    satellite.DrawCubeSat(time, 0, 1);

    % 捕获当前帧
    frame = getframe(gcf);
    writeVideo(video, frame); % 写入视频
end
% 关闭视频对象
close(video);
disp('动画生成完成，已保存为文件: satellite_orbit_animation.avi');


function [r_list, v_list, e_list, w_list]...
       = simulation(r0, v0, e0, w0, time, dt)
    % 微分方程求解辅助函数: 处理多变量
    % 欧拉角转换为四元数
    q0 = EulerToQuaternion(e0);
    y0 = [r0,v0,q0,w0];
    
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

    r = y(1:3);
    v = y(4:6);
    q = y(7:10);
    w = y(11:13);

    % 轨道动力学
    [dr_dt, dv_dt] = satellite.OrbitDynamics(r,v,earth);
    % 姿态动力学
    [dq_dt, dw_dt] = satellite.AttitudeDynamics(q, w);
    
    dy_dt = [dr_dt, dv_dt, dq_dt, dw_dt];
end