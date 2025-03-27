classdef Earth
    properties
        Mu % 地球标准引力参数(km^3/s^2)
        Re % 地球半径(km)
        J2 % 地球椭圆假设引力J2项
        RotationVelocity % 地球自转角速度(rad/s)
        StartDate % 仿真开始日期"YYYY-MM-DD-HH-MM-SS"
        Dt % 仿真时间颗粒度(s)
        Time % 仿真总时间(s)
        Length % 仿真步数
        AlphaList % 春分点和Greenwich子午线之间夹角(rad)
        % 添加新属性用于存储地球显示对象
        EarthSurface  % 地球表面对象句柄
        TextureData   % 地球纹理数据
        % 添加原始球体坐标数据
        OriginalSphereX
        OriginalSphereY
        OriginalSphereZ
    end
    methods 
        % 初始化方法
        function obj = Earth(start_date, ...
                             time, dt)
            obj.Mu = 398600;
            obj.Re = 6371;
            obj.J2 = 1.08263*10^(-3);
            obj.RotationVelocity = 7.29211515e-5;

            obj.StartDate = start_date;
            obj.Dt = dt;
            obj.Time = time;
            obj.Length = floor(time/dt);
       
            % 仿真地球自转
            obj.AlphaList = obj.CalculateEarthRotation();
            
            % 预加载纹理数据
            obj.TextureData = imread('earth_texture.jpg');
        end
        % 计算 Modified Julian Date (MJD)
        function mjd = DateToMJD(~, date_str)
            % 输入日期字符串格式 "YYYY-MM-DD-HH-MM-SS"

            date = datetime(date_str, 'InputFormat', 'yyyy-MM-dd-HH-mm-ss');
            JD = juliandate(date);
            mjd = JD - 2400000.5;
        end
        % 计算初始时刻地球春分点和Greenwich子午线之间夹角
        function alpha0 = CalculateInitialAlpha(obj)

            mjd = obj.DateToMJD(obj.StartDate);
            alpha0 = (280.46061837+360.98564736629*mjd ...
                    + 0.000388*(mjd/36525)^2)*pi/180;
            % 确保角度在 [0, 2π] 范围内
            alpha0 = mod(alpha0, 2 * pi);
        end
        % 仿真地球自转
        function alpha_list = CalculateEarthRotation(obj)

            dt = obj.Dt;
            length = obj.Length;
            alpha_list = zeros(obj.Length,1);
            alpha_list(1) = obj.CalculateInitialAlpha();
            rotation_velocity = obj.RotationVelocity;
            
            for i = 2:length
                delta_alpha = dt * rotation_velocity;
                alpha_list(i) = mod(alpha_list(i-1) + delta_alpha,2*pi);
            end
        end
        % 计算r(km)位置的地球磁场强度
        function B = MagneticField(~, r)

            x = r(1)*1000;
            y = r(2)*1000;
            z = r(3)*1000;
            % 地球磁偶极矩
            m = [7.94e22, 0, 0]; % 单位：Am^2，地球磁偶极矩，大致沿地理北极方向
            
            % 真空磁导率 (T·m/A)
            mu_0 = 4 * pi * 1e-7; 
            
            % 计算距离 r 和单位向量 r_hat
            r = sqrt(x^2 + y^2 + z^2);  % 距离
            r_hat = [x, y, z] / r;  % 单位向量
            
            % 计算磁场
            m_dot_r = dot(m, r_hat);  % 偶极矩与单位向量的点积
            B = (mu_0 / (4 * pi)) * ((3 * m_dot_r * r_hat - m) / r^3);  % 磁场公式
        end
        % 初始化地球显示对象
        function obj = InitializeEarthPlot(obj, re_scale)

            [x_sphere, y_sphere, z_sphere] = sphere(50);
            
            % 保存原始坐标数据
            obj.OriginalSphereX = re_scale * x_sphere;
            obj.OriginalSphereY = re_scale * y_sphere;
            obj.OriginalSphereZ = re_scale * z_sphere;
            
            % 创建地球表面对象
            obj.EarthSurface = surf(obj.OriginalSphereX, ...
                                  obj.OriginalSphereY, ...
                                  obj.OriginalSphereZ, ...
                                  'FaceColor', 'texturemap', ...
                                  'CData', obj.TextureData, ...
                                  'EdgeColor', 'none');
        end
        % 更新地球旋转状态
        function UpdateEarthRotation(obj, time)

            dt = obj.Dt;
            alpha_list = obj.AlphaList;
            
            i = floor(time / dt);
            alpha = alpha_list(i);
            
            % 绕z轴旋转矩阵
            Rz = [cos(alpha), -sin(alpha), 0;
                  sin(alpha), cos(alpha), 0;
                  0, 0, 1];
            
            % 使用原始坐标数据
            coords = [reshape(obj.OriginalSphereX, [], 1), ...
                     reshape(obj.OriginalSphereY, [], 1), ...
                     reshape(obj.OriginalSphereZ, [], 1)]';
            
            % 应用旋转
            rotated_coords = Rz * coords;
            
            % 更新表面数据
            [rows, cols] = size(obj.OriginalSphereX);
            set(obj.EarthSurface, 'XData', reshape(rotated_coords(1,:), rows, cols));
            set(obj.EarthSurface, 'YData', reshape(rotated_coords(2,:), rows, cols));
            set(obj.EarthSurface, 'ZData', reshape(rotated_coords(3,:), rows, cols));
        end
    end
end