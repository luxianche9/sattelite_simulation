function DrawCube(edge_length)
    % 绘制一个方形透明框, 固定动画场景大小
    % edge_length: 方形框边长
    
    half_length = edge_length / 2;
    % 顶点
    vertices = [
        -1, -1, -1;
         1, -1, -1;
         1,  1, -1;
        -1,  1, -1;
        -1, -1,  1;
         1, -1,  1;
         1,  1,  1;
        -1,  1,  1
    ] * half_length;
    % 边
    edges = [
        1, 2; 2, 3; 3, 4; 4, 1; % 底面
        5, 6; 6, 7; 7, 8; 8, 5; % 顶面
        1, 5; 2, 6; 3, 7; 4, 8  % 垂直面
    ];
    % 绘制
    for i = 1:size(edges, 1)
        h = plot3([vertices(edges(i, 1), 1), vertices(edges(i, 2), 1)], ...
                  [vertices(edges(i, 1), 2), vertices(edges(i, 2), 2)], ...
                  [vertices(edges(i, 1), 3), vertices(edges(i, 2), 3)], ...
                  'k-', 'LineWidth', 1.5);
        set(h, 'Color', [0.9, 0.9, 0.9, 0]);
    end
end