function [r, v] = ElementsToRV(a,e,i,Omega,omega,theta)
    % 将轨道六要素转化为球心惯性坐标系下的位置和速度
    % a: 半长轴
    % e: 偏心率
    % i: 轨道倾角(rad)
    % Omega: 升交点赤经(rad)
    % omega: 近地点辐角(rad)
    % theta: 真近点角(rad)
    % r: 位置[x,y,z](km)
    % v: 速度[v_x, v_y, v_z](km/s)
    
    % 地球引力常数(km^3/s^2)
    mu = 398600;

    % 半通径
    p = a * (1 - e^2);

    % 地心轨道坐标系中的位置矢量 (km)
    r_o = [p * cos(theta) / (1 + e * cos(theta));
           p * sin(theta) / (1 + e * cos(theta));
           0];

    % 地心轨道坐标系中的速度矢量 (km/s)
    v_o = [-sqrt(mu / p) * sin(theta);
            sqrt(mu / p) * (e + cos(theta));
            0];

    % 旋转矩阵 (从地心轨道坐标系到赤道惯性坐标系)
    R = rotz(Omega)*rotx(i)*rotz(omega);

    % 赤道惯性坐标系中的位置和速度
    r = (R * r_o)';
    v = (R * v_o)';

    % 显示计算结果
    % fprintf('由轨道六要素计算赤道惯性坐标系下的初始位置和速度:\n');
    % fprintf('  初始位置 (km): [%f, %f, %f]\n', r(1), r(2), r(3));
    % fprintf('  初始速度 (km/s): [%f, %f, %f]\n', v(1), v(2), v(3));
end