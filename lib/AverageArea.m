function A = AverageArea(shape)
    % 计算立方体的平均表面积
    % shape: 外形[length, width, height](m)

    length = shape(1);
    width = shape(2);
    height = shape(3);

    TotalSurfaceArea = 2 * (length * width ...
                          + width * height ...
                          + length * height);
    A = TotalSurfaceArea/6;
end