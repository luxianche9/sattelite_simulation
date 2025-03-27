% 测试
% Class Earth

clear;
clc;

start_date = "2024-12-15-06-31-00";% 仿真开始日期时间
dt = 61;% 1min
time = 60*60*24;% 1day
length = floor(time/dt);

earth = Earth(start_date, time, dt);

figure;
title('可视化地球自转');
hold on;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
rotate3d on;
time = 0;
for n = 1:length
    cla;
    earth.DrawEarth(time, 10)
    pause(0.01);
    time = time + dt;
end