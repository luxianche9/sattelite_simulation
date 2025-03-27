% 测试 IsInShade 函数
% 生成位于地球表面附近的点，模拟一个圆周

clear; clc;

% 参数设置
earth_radius = 6371; % 地球半径，单位：km
altitude = 1000; % 卫星高度，单位：km
orbit_radius = earth_radius + altitude; % 轨道半径

num_points = 100; % 圆周上的点数
theta = linspace(0, 2*pi, num_points); % 圆周角度
r_points = orbit_radius * [cos(theta); sin(theta); zeros(1, num_points)]; % 生成圆周上的点

% 太阳方向（假设固定）
q_sun = [1, 0, 0]; % 太阳方向为正 x 方向

% 计算是否在地影区
results = zeros(1, num_points); % 存储结果
for i = 1:num_points
    r = r_points(:, i);
    results(i) = IsInShade(q_sun, r);
end


% 可视化结果
figure;
hold on;
axis equal;

% 绘制地球
theta_earth = linspace(0, 2*pi, 100);
earth_x = earth_radius * cos(theta_earth);
earth_y = earth_radius * sin(theta_earth);
fill(earth_x, earth_y, 'b', 'FaceAlpha', 0.1, 'EdgeColor', 'b', 'DisplayName','地球'); % 地球区域

% 绘制卫星轨道
plot(r_points(1, :), r_points(2, :), 'k--', 'LineWidth', 1, 'DisplayName','轨道');
% 绘制结果点
for i = 1:num_points
    if results(i) == 1
        plot(r_points(1, i), r_points(2, i), 'ro', 'DisplayName','地影区'); % 地影区点
    else
        plot(r_points(1, i), r_points(2, i), 'go', 'DisplayName','非地影区'); % 非地影区点
    end
end

% 标注
title('地影区测试');
xlabel('X (km)');
ylabel('Y (km)');
hold off;


function in_shade = IsInShade(q, r)
    q = q ./ norm(q);
    phi = acos(dot(q,r)/norm(r));
    in_shade = 0;
    if (phi > pi/2) && (norm(r)*sin(phi) < 6371)
        in_shade = 1;
    end
end
