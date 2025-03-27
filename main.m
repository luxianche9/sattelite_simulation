clear;
clc;

% 仿真参数设置, 卫星初始条件设置
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 仿真时间
start_date = "2024-12-24-13-30-00";
simulation_time = 8000;%(s) 2*pi*sqrt(a^3/earth.Mu)
dt = 1;%(s)
pause_time = 0.01;%动画

% 轨道
% r0: 初始位置[x,y,z](km)
% v0: 初始速度[v_x, v_y, v_z](km/s)
a = 9000; % 半长轴
e = 0; % 偏心率
i = deg2rad(0); % 轨道倾角
Omega = deg2rad(20); % 升交点赤经
omega = deg2rad(0); % 近地点辐角
theta = deg2rad(0); % 真近点角
[r0, v0] = ElementsToRV(a,e,i,Omega,omega,theta);

% 姿态
% e0: 初始姿态[roll, pitch, yaw](rad)
% w0: 初始角速度[w_x, w_y, w_z](rad/s)
% W0: 飞轮初始角速度(rad/s)
e0 = [0, 0, 0];
w0 = [0, 0, 0];
W0 = [0, 0, 100];

% 生成仿真对象
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global earth;
global satellite;
global sun;
earth = Earth(start_date, simulation_time, dt);
satellite = Satellite(simulation_time, dt);
sun = Sun(start_date, dt, simulation_time);


% 数值求解轨道, 姿态
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[r_list, v_list, e_list, w_list, W_list] = ...
    simulation(r0, v0, e0, w0, W0, simulation_time, dt);
satellite.PositionList = r_list;
satellite.VelocityList = v_list;
satellite.AttitudeList = e_list;
satellite.AngularVelocityList = w_list;
satellite.WheelSpeedList = W_list;


% 生成图表
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t = linspace(0,simulation_time,simulation_time/dt);
colors = {'#81BFDA', '#15B392', '#F72C5B', '#B1F0F7', '#D2FF72', '#FF748B'};
% 卫星姿态数据
figure
hold on
grid on
title('卫星姿态');
xlabel('时间/s');
ylabel('姿态/deg');
% 卫星姿态
plot(t, satellite.AttitudeList(:,1)*180/pi, 'Color', colors{1}, 'LineStyle', '-');
plot(t, satellite.AttitudeList(:,2)*180/pi, 'Color', colors{2}, 'LineStyle', '-');
plot(t, satellite.AttitudeList(:,3)*180/pi, 'Color', colors{3}, 'LineStyle', '-');
% 目标姿态
plot(t, satellite.TargetAttitudeList(:,1)*180/pi, 'Color', colors{4}, 'LineStyle', '--');
plot(t, satellite.TargetAttitudeList(:,2)*180/pi, 'Color', colors{5}, 'LineStyle', '--');
plot(t, satellite.TargetAttitudeList(:,3)*180/pi, 'Color', colors{6}, 'LineStyle', '--');
legend('roll', 'pitch', 'yaw', 'roll target', 'pitch target', 'yaw target');
hold off

% 卫星角速度数据
figure
hold on
grid on
title('卫星角速度');
xlabel('时间/s');
ylabel('角速度/(deg/s)');
% 卫星角速度
plot(t, satellite.AngularVelocityList(:,1)*180/pi, 'Color', colors{1}, 'LineStyle', '-');
plot(t, satellite.AngularVelocityList(:,2)*180/pi, 'Color', colors{2}, 'LineStyle', '-');
plot(t, satellite.AngularVelocityList(:,3)*180/pi, 'Color', colors{3}, 'LineStyle', '-');
% 目标角速度
plot(t, satellite.TargetAngularVelocityList(:,1)*180/pi, 'Color', colors{4}, 'LineStyle', '--');
plot(t, satellite.TargetAngularVelocityList(:,2)*180/pi, 'Color', colors{5}, 'LineStyle', '--');
plot(t, satellite.TargetAngularVelocityList(:,3)*180/pi, 'Color', colors{6}, 'LineStyle', '--');
legend('w_x', 'w_y', 'w_z', 'w_x target', 'w_y target', 'w_z target');
hold off

% 飞轮转速数据
figure
hold on
grid on
title('飞轮转速');
xlabel('时间/s');
ylabel('转速/rpm');
plot(t, satellite.WheelSpeedList(:,1)*60/(2*pi), 'Color', colors{1}, 'LineStyle', '-');
plot(t, satellite.WheelSpeedList(:,2)*60/(2*pi), 'Color', colors{2}, 'LineStyle', '-');
plot(t, satellite.WheelSpeedList(:,3)*60/(2*pi), 'Color', colors{3}, 'LineStyle', '-');
legend('W_x', 'W_y', 'W_z');
hold off

% 磁力矩器数据
figure
hold on
grid on
title('磁力矩大小');
xlabel('时间/s');
ylabel('磁力矩/(N*m)');
plot(t, satellite.MagneticTorqueList(:,1), 'Color', colors{1}, 'LineStyle', '-');
plot(t, satellite.MagneticTorqueList(:,2), 'Color', colors{2}, 'LineStyle', '-');
plot(t, satellite.MagneticTorqueList(:,3), 'Color', colors{3}, 'LineStyle', '-');
legend('W_x', 'W_y', 'W_z');
hold off

% 卫星制导状态参数
% 状态(spin)
figure
hold on
grid on
title('是否自旋搜索太阳');
xlabel('时间/s');
plot(t, satellite.StatusList(:,2), 'k', 'LineStyle', '-');
legend('控制自旋');
hold off;


% 生成动画
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 创建窗口
figure;
title('可视化卫星轨道');
hold on;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
rotate3d on;

re_scale = 2;
position_scale = 5;
h_scale = 4;
shape_scale = 1;

satellite = satellite.InitializeCubeSat(shape_scale);
earth = earth.InitializeEarthPlot(re_scale);
sun = sun.InitializeShadow(re_scale, h_scale);
sun = sun.InitializeLight(position_scale);
DrawCube(4);
for t = dt:dt:simulation_time
    earth.UpdateEarthRotation(t);
    sun.UpdateShadow(t);
    sun.UpdateLight(t, position_scale);
    satellite.UpdateCubeSat(t, re_scale);
    satellite.DrawOrbit(t, re_scale);

    drawnow;
    pause(pause_time);
end