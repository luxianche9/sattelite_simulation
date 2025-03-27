clear;
clc;

start_date = "2024-12-15-06-31-00";% 仿真开始日期时间
dt = 60*60*24;% 1day
time = 60*60*24*365;% 1year
length = floor(time/dt);

sun = Sun(start_date, dt, time);
earth = Earth(start_date, dt, time);
simulation = Simulation();

figure;
title('可视化太阳公转');
hold on;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
rotate3d on;
time = 0;
for n = 1:length
    cla;
    simulation.DrawFrameCube(12);
    sun.DrawSun(time, 1, 5);
    sun.DrawSunLight(time, 2,1);
    sun.DrawShad(time, 1, 5);
    earth.DrawEarth(time, 1);
    pause(0.02);
    time = time + dt;
end