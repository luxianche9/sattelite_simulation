function I = InertiaMatrix(mass, shape)
    % 计算质量均匀分布立方体的转动惯量矩阵
    % mass: 质量(kg)
    % shape: 外形[length, width, height](m)

    length = shape(1);
    width = shape(2);
    height = shape(3);

    I_xx = (1/12) * mass * (height^2 + width^2);
    I_yy = (1/12) * mass * (length^2 + height^2);
    I_zz = (1/12) * mass * (length^2 + width^2);
    
    % 体坐标系下的转动惯量矩阵
    I = [I_xx, 0, 0;
         0, I_yy, 0;
         0, 0, I_zz];
end