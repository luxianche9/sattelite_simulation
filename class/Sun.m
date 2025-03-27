classdef Sun
    properties
        StartDate % 仿真开始日期"YYYY-MM-DD-HH-MM-SS"
        Dt % 仿真时间颗粒度(s)
        Time % 仿真总时间(s)
        Length % 仿真步数
        QList % 地球惯性坐标系下太阳位置单位矢量1x3
        ShadowCylinder  % 地影圆柱对象句柄
        LightSource     % 光源对象句柄
        % 存储原始圆柱数据
        OriginalCylinderX
        OriginalCylinderY
        OriginalCylinderZ
    end
    methods
        % 构造方法
        function obj = Sun(start_date, ...
                             dt, time)
            obj.StartDate = start_date;
            obj.Dt = dt;
            obj.Time = time;
            obj.Length = floor(time/dt);
       
            % 仿真太阳公转
            obj.QList = obj.CalculateQ();
        end
        % 计算 Modified Julian Date (MJD)      
        function mjd = DateToMJD(~, date_str)
            % 输入日期字符串格式 "YYYY-MM-DD-HH-MM-SS"

            date = datetime(date_str, 'InputFormat', 'yyyy-MM-dd-HH-mm-ss');
            JD = juliandate(date);
            mjd = JD - 2400000.5;
        end
        % 计算太阳太阳单位矢量
        function q_list = CalculateQ(obj)
            length = obj.Length;
            dt = obj.Dt;
            q_list = zeros(length,3);

            mjd = obj.DateToMJD(obj.StartDate);
            for i = 1:length
                rtd = 180/pi;
                eps = (23.439-0.0000004*mjd)/rtd; % 黄赤交角
                eh = 0.016709; % 偏心率
                L = (280.461 + 0.9856474*mjd)/rtd; % 太阳平经度
                Mh = (357.528 + 0.9856003*mjd)/rtd; % 太阳平近点角
                Lam = L+2*eh*sin(Mh)+1.25*eh^2*sin(2*Mh); % 太阳黄经

                alpha_h = atan2(cos(eps)*sin(Lam), cos(Lam));%太阳赤经
                del_h = asin(sin(eps)*sin(Lam));%太阳赤纬

                q = [cos(alpha_h)*cos(del_h), ...
                     sin(alpha_h)*cos(del_h), ...
                     sin(del_h)];
                q_list(i, :) = q;

                mjd = mjd + dt/86400;
            end
        end        
        % 初始化地影圆柱显示对象
        function obj = InitializeShadow(obj, re_scale, h_scale)

            [X, Y, Z] = cylinder([re_scale, re_scale], 50);
            Z = -Z * h_scale;
            
            % 保存原始圆柱数据
            obj.OriginalCylinderX = X;
            obj.OriginalCylinderY = Y;
            obj.OriginalCylinderZ = Z;
            
            % 创建地影圆柱对象
            obj.ShadowCylinder = surf(X, Y, Z, ...
                                    'FaceAlpha', 0.3, ...
                                    'EdgeColor', 'none', ...
                                    'FaceColor', [0.5, 0.5, 0.5]);
        end
        % 初始化光源
        function obj = InitializeLight(obj, position_scale)
            
            position = obj.QList(1, :) * position_scale;
            obj.LightSource = light('Position', position, ...
                                  'Style', 'local');
            lighting gouraud;
            material shiny;
        end
        % 更新地影圆柱
        function UpdateShadow(obj, time)
            dt = obj.Dt;
            q_list = obj.QList;
            i = floor(time / dt);
            q = q_list(i, :);
            
            % 创建旋转矩阵
            zAxis = [0, 0, 1];
            v = cross(zAxis, q);
            c = dot(zAxis, q);
            s = norm(v);
            
            Vx = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
            R = eye(3) + Vx + Vx^2 * ((1 - c) / s^2);
            
            % 应用旋转到原始圆柱坐标
            vertices = [reshape(obj.OriginalCylinderX, [], 1), ...
                       reshape(obj.OriginalCylinderY, [], 1), ...
                       reshape(obj.OriginalCylinderZ, [], 1)];
            
            rotated_vertices = (R * vertices')';
            
            % 更新显示
            [rows, cols] = size(obj.OriginalCylinderX);
            set(obj.ShadowCylinder, 'XData', reshape(rotated_vertices(:,1), rows, cols));
            set(obj.ShadowCylinder, 'YData', reshape(rotated_vertices(:,2), rows, cols));
            set(obj.ShadowCylinder, 'ZData', reshape(rotated_vertices(:,3), rows, cols));
        end
        % 更新光源位置
        function UpdateLight(obj, time, position_scale)
            dt = obj.Dt;
            q_list = obj.QList;
            i = floor(time / dt);
            q = q_list(i, :);
            
            position = q * position_scale;
            set(obj.LightSource, 'Position', position);
        end

        % function DrawSun(obj, time, re_scale, position_scale)
        %     % time: 从仿真开始经过time(s)后的太阳
        %     % re_scale: 绘制的太阳的半径大小
        %     % pos_scale: 绘制的地日距离
        %     % q: 当前太阳位置单位矢量
        %     dt = obj.Dt;
        %     q_list = obj.QList;
        %     i = floor(time / dt);
        %     q = q_list(i, :);
        % 
        %     position = q * position_scale;
        % 
        %     % 生成太阳表面网格, 增加位置偏移
        %     [x_sphere, y_sphere, z_sphere] = sphere(50);
        %     x_sphere = x_sphere*re_scale + position(1);
        %     y_sphere = y_sphere*re_scale + position(2);
        %     z_sphere = z_sphere*re_scale + position(3);
        % 
        %     surf(x_sphere, y_sphere, z_sphere, ...
        %          'FaceColor', '#EF8900','EdgeColor', 'none');
        % 
        %     % 设置光照
        %     light('Position', position, 'Style', 'local');
        %     lighting gouraud;
        %     material shiny;
        % end
        % 
        % function DrawShad(obj, time, re_scale, h_scale)
        %     % 绘制一个圆柱代表地影区
        %     % q: 太阳位置单位矢量
        %     % re_scale: 绘制的地球的半径
        %     % h_scale: 绘制的圆柱的高度
        %     dt = obj.Dt;
        %     q_list = obj.QList;
        %     i = floor(time / dt);
        %     q = q_list(i, :);
        % 
        %     n = 50; % 圆柱网格
        %     [X, Y, Z] = cylinder([re_scale, re_scale], n);
        %     Z = - Z * h_scale;% 圆柱的高度
        % 
        %     % 创建方向矢量 q 的旋转矩阵
        %     zAxis = [0, 0, 1]; % z 轴方向
        %     v = cross(zAxis, q); % 计算旋转轴
        %     c = dot(zAxis, q);   % 计算旋转角度的余弦值
        %     s = norm(v);         % 计算旋转角度的正弦值
        % 
        %     % 构造旋转矩阵
        %     Vx = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
        %     R = eye(3) + Vx + Vx^2 * ((1 - c) / s^2);
        % 
        %     % 将圆柱从标准方向旋转到 q 的方向
        %     for i = 1:size(X, 1)
        %         for j = 1:size(X, 2)
        %             rotated = R * [X(i, j); Y(i, j); Z(i, j)];
        %             X(i, j) = rotated(1);
        %             Y(i, j) = rotated(2);
        %             Z(i, j) = rotated(3);
        %         end
        %     end
        %     % 绘制圆柱
        %     surf(X, Y, Z, ...
        %         'FaceAlpha', 0.3, 'EdgeColor', 'none', ...
        %         'FaceColor', [0.5, 0.5, 0.5]);
        % end
        % 
        % function DrawSunLight(obj, time, distance, length)
        %     % time: 当前时间
        %     % distance: 箭头起点距离原点的长度
        %     % length: 箭头长度
        %     dt = obj.Dt;
        %     q_list = obj.QList; % 单位矢量列表
        %     i = floor(time / dt); % 根据时间索引单位矢量
        %     q = q_list(i, :); % 获取当前时间的单位矢量方向
        % 
        %     % 箭头起点和方向矢量
        %     tail = distance * q; % 起点
        %     head = - length * q;   % 方向矢量
        % 
        %     % 使用 quiver3 绘制箭头
        %     quiver3(tail(1), tail(2), tail(3), ...
        %             head(1), head(2), head(3), ...
        %             0, 'Color', '#FFbb00', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        % end
        % 
        % function DrawLight(obj, time, position_scale)
        %     dt = obj.Dt;
        %     q_list = obj.QList;
        %     i = floor(time / dt);
        %     q = q_list(i, :);
        % 
        %     position = q * position_scale;
        %     light('Position', position, 'Style', 'local');
        %     lighting gouraud;
        %     material shiny;
        % end


    end
end