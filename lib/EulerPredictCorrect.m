function y = EulerPredictCorrect(time, dt, y0, f)
    % 欧拉预测矫正法求解微分方程
    % y为行向量
    % dy/dt = f(t,y)
    % y(0) = y0
    % 0 < t < time
    % y0: 行向量
    % y: [y0; y1; y2; ...]

    length = floor(time/dt);
    ysize = size(y0);
    ysize = ysize(2);

    t = (0:dt:time)';
    y = zeros(length, ysize);
    y(1,:) = y0;

    for i = 1 : length - 1
        K1 = f(t(i), y(i,:));
        K2 = f(t(i) + dt, y(i,:) + dt * K1);
        y(i+1,:) = y(i,:)  + dt/2 * (K1 + K2);
    end
end