classdef Satellite
    properties
        % 常量
        Mass % 质量(kg)
        Shape % 形状[长,宽,高](m)
        InertiaMatrix % 转动惯量矩阵(kg*m^2)
        Cd % 阻力系数
        Area % 迎面面积(m^2)
        WheelSpeedLimit % 飞轮转速限制(rad/s)
        WheelSpeedAccelerationLimit % 飞轮角加速度限制(rad/s^2)
        WheelInertia % 飞轮转动惯量(kg*m^2);
        MagneticMomentLimit % 最大磁偶极矩(A*m^2)
        SensorField % 太阳敏视场
        SpinVelocity % 自转角速度(rad/s)
        Kp % 比例控制器系数
        Kd % 微分控制器系数
        Dt % 仿真时间颗粒度(s)
        Time % 仿真总时长(s)
        Length % 仿真总步长
        % 输出变量
        PositionList % 位置列表[x, y, z](km)
        VelocityList % 速度列表[vx, vy, vz](km/s)
        AngularVelocityList % 角速度列表[wx, wy, wz](rad/s)
        AttitudeList % 姿态列表[roll, pitch, yaw](rad)
        % 状态变量
        WheelSpeedList % 飞轮转速[wx, wy, wz](rad/s)
        StatusList % 是否在地影区, 是否旋转, 是否检测到太阳
                   % [in_shade, spin, detect_x, detect_z]
        TargetAttitudeList % 目标姿态[roll, pitch, yaw](rad)
        TargetAngularVelocityList % 目标转动角速度[wx, wy, wz](rad/s)
        MagneticTorqueList % 消旋磁力矩大小[M_x, M_y, M_z](N*m)
        % 添加显示对象属性
        CubeSatBody      % 卫星本体对象句柄
        SensorConeX      % X轴传感器视场锥对象句柄
        SensorConeZ      % Z轴传感器视场锥对象句柄
        % 存储原始形状数据
        OriginalVertices     % 卫星顶点原始数据
        OriginalConeXVertex  % X轴视场锥顶点
        OriginalConeXBase    % X轴视场锥底面点
        OriginalConeZVertex  % Z轴视场锥顶点
        OriginalConeZBase    % Z轴视场锥底面点
    end

    methods
        function obj = Satellite(time, dt)
            obj.Mass = 12;
            obj.Shape = [22.6,10,34]*1E-2;
            obj.InertiaMatrix = InertiaMatrix(obj.Mass, obj.Shape);
            obj.Cd = 2.2;
            obj.Area = AverageArea(obj.Shape);
            obj.WheelSpeedLimit = 6000 * 2 * pi / 60;%6000 rpm
            obj.WheelSpeedAccelerationLimit = 83.3;% 1E-3 N*m
            obj.WheelInertia = 1.2E-5;
            obj.MagneticMomentLimit = 0.4;
            obj.SensorField = 2 * pi / 3;
            obj.SpinVelocity = 0.01;
            t = 100;
            obj.Kp = -1.0467e+04*(4/t)*(4/t);
            obj.Kd = -1.0467e+04*2*(4/t);
            obj.Time = time;
            obj.Dt = dt;
            obj.Length = floor(time/dt);

            obj.PositionList = zeros(obj.Length, 3);
            obj.VelocityList = zeros(obj.Length, 3);
            obj.AngularVelocityList = zeros(obj.Length, 3);
            obj.AttitudeList = zeros(obj.Length, 3);

            obj.WheelSpeedList = zeros(obj.Length, 3);
            obj.StatusList = zeros(obj.Length, 4);
            obj.TargetAttitudeList = zeros(obj.Length, 3);
            obj.TargetAngularVelocityList = zeros(obj.Length, 3);
            obj.MagneticTorqueList = zeros(obj.Length, 3);
        end
        
        % 轨道动力学
        function [dr_dt, dv_dt] = OrbitDynamics(obj, r, v, earth)
            % 卫星轨道动力学
            % r: 位置[x,y,z](km)
            % v: 速度[v_x, v_y, v_z](km/s)
            % dr_dt: dr/dt 行向量
            % dv_dt: dv/dtt 行向量

            % 引力加速度
            mu = earth.Mu;
            r_mag = norm(r);
            a_gravity = -mu * r(:) / r_mag^3;
            % 非球型引力摄动
            J2 = earth.J2;
            re = earth.Re;
            a_J2 = [-2/3*J2*re^2*mu*r(1)/r_mag^5*(1-5*r(3)^2/r_mag^2);...
                -2/3*J2*re^2*mu*r(2)/r_mag^5*(1-5*r(3)^2/r_mag^2);...
                -2/3*J2*re^2*mu*r(3)/r_mag^5*(3-5*r(3)^2/r_mag^2)];
            % 空气阻力摄动
            rho0 = 1.225; % 海平面大气密度(kg/m^3)
            scale_height = 8.5; % (km)
            m = obj.Mass;
            area = obj.Area;
            cd = obj.Cd;
            h = r_mag - re;
            rho = rho0 * exp(-h / scale_height);
            drag_force = 0.5 * cd * area * rho * norm(v(:))^2;
            a_drag = -drag_force / m * (v(:) / norm(v(:)));
            
            % 轨道动力学方程
            dv_dt = (a_gravity + a_drag + a_J2)';
            dr_dt = v;
        end

        % 姿态动力学
        function [dq_dt, dw_dt] = AttitudeDynamics(obj, q, w, dW_dt, W, M_m, M_d)
            % q: 四元数[w, x, y, z]
            % w: 卫星体坐标系下转动角速度[wx, wy, wz]
            % dW_dt: dW/dt 飞轮角加速度(rad/s^2)
            % W: 飞轮角速度(rad/s)
            % M_m: 磁力矩
            % M_d: 干扰力矩

            % 单位化四元数
            q = QuatNormalize(q);
            
            % 转动惯量矩阵
            I = obj.InertiaMatrix;
            Ix = I(1,1);
            Iy = I(2,2);
            Iz = I(3,3);
            % 飞轮转动惯量
            Iw = obj.WheelInertia;

            % 飞轮控制力矩
            M_c = [Iw*(dW_dt(1)+W(3)*w(2)-W(2)*w(3)), ...
                   Iw*(dW_dt(2)+W(1)*w(3)-W(3)*w(1)), ...
                   Iw*(dW_dt(3)+W(2)*w(1)-W(1)*w(2))];

            % 合并所有力矩
            M_total = + M_d + M_m + M_c;

            % 姿态动力学方程
            dw1_dt = (-M_total(1) - (Iz - Iy) * w(2) * w(3)) / Ix;
            dw2_dt = (-M_total(2) - (Ix - Iz) * w(1) * w(3)) / Iy;
            dw3_dt = (-M_total(3) - (Iy - Ix) * w(1) * w(2)) / Iz;

            dw_dt = [dw1_dt, dw2_dt, dw3_dt];
            dq_dt = 0.5 * QuatMultiply(q, [0, w]);
        end

        % 制导
        function [e_t, w_t, status] = Guidance(obj, q, r, e)
            % q: 太阳位置(单位矢量)
            % r: 卫星位置[x,y,z](km)
            % e: 卫星姿态[roll, pitch, yaw](rad)
            % e_t 卫星目标姿态[roll,pitch,yaw](rad)
            % w_t 卫星目标角速度[wx, wy, xz](rad/s)
            % status 卫星状态[in_shade, spin, detect_x, detect_z]
           
            % 制导逻辑
            in_shade = obj.IsInShade(q, r);
            [detect_x,detect_z] = obj.SunSensor(e, q);
            detect_sun = detect_x || detect_z;

            persistent search_phase accumulated_angle
            % 初始化搜索阶段和计数器
            if isempty(search_phase)
                search_phase = 1;  % 1: x轴搜索, 2: z轴搜索
                accumulated_angle = 0;
            end
            
            if in_shade || ((~in_shade) && (~detect_sun))       
                % 更新累积角度
                accumulated_angle = accumulated_angle + obj.SpinVelocity * obj.Dt;
                
                % 如果完成一圈旋转（180度），切换搜索阶段
                % 欧拉预测矫正法会在一个dt中调用两次guidance, 角度累计会翻倍
                if accumulated_angle >= 2*pi
                    search_phase = mod(search_phase, 2) + 1;
                    accumulated_angle = 0;  % 重置累积角度
                end
                
                % 根据搜索阶段设置旋转轴
                if search_phase == 1
                    % 绕x轴旋转
                    k = [1, 0, 0];
                else
                    % 绕z轴旋转
                    k = [0, 0, 1];
                end
                
                w = obj.SpinVelocity;    % 角速度大小
                
                % 设置角速度矢量
                w_t = k * w;
                
                % 计算下一时刻的目标姿态
                % 当前姿态欧拉角转四元数 [w,x,y,z]
                q_current = EulerToQuaternion(e);
                
                % 计算微小旋转的四元数
                dt = obj.Dt;
                theta = w * dt / 2;  % 注意：四元数旋转角度是欧拉角的一半
                q_rot = [cos(theta), k*sin(theta)];
                
                % 四元数乘法计算新姿态
                q_new = QuatMultiply(q_rot, q_current);
                
                % 四元数转欧拉角
                e_t = QuaternionToEuler(q_new);
                
                spin = 1;
                detect_z = 0;
                detect_x = 0;
            else
                % 检测到太阳时的逻辑保持不变
                w_t = [0,0,0];
                e_t = obj.TargetAttitude(q);
                spin = 0;
                
                % 重置所有状态
                search_phase = 1;
                accumulated_angle = 0;
            end
            
            status = [in_shade, spin, detect_x, detect_z];
        end

        
        % 控制
        function [dW_dt, M_m] = Control(obj, e, w, e_t, w_t, W, B)
            % 使用 PD 控制器计算飞轮转速的微分以调整卫星姿态
            % e: 姿态角 [roll, pitch, yaw] (rad)
            % w: 绕本体坐标系的角速度 (rad/s)
            % e_t: 目标姿态角
            % w_t: 目标转动角速度
            % W: 飞轮角速度(rad/s)
            % dW_dt: dW/dt飞轮角加速度 (rad/s^2)
            % B: 惯性坐标系下的磁场强度(T)
            
            % 误差
            e_e = e_t - e;
            w_e = w_t - w; 
            % 计算 PD 控制器输出
            dW_dt = obj.Kp * e_e + obj.Kd * w_e;
    
            % 飞轮最大角加速度限制
            Alimit = obj.WheelSpeedAccelerationLimit;
            dW_dt = max(min(dW_dt, Alimit), -Alimit);
            % 飞轮最大角速度限制
            Wlimit = obj.WheelSpeedLimit;
            for n = 1:3
                if abs(W(n)) >= Wlimit
                    dW_dt(n) = 0;
                end
            end

            % 磁力矩器卸载飞轮
            % 本体坐标系下地磁场投影
            R = eul2rotm(e, "XYZ");
            B = (R*B')';
            % 计算磁偶极矩
            % 使用 A × B = -W 的关系
            % 由于 M = A × B，我们希望 M 与 W 方向相反以卸载动量
            % 因此 A = k * (B × W) / |B|，其中k是增益系数
            k = 0.1; % 卸载增益系数
            
            if norm(B) > 1e-10  % 防止除零
                A = k * cross(B, W) / norm(B);
            else
                A = zeros(1,3);
            end
            % 磁矩限制
            A_limit = obj.MagneticMomentLimit;
            A_norm = norm(A);
            if A_norm > A_limit
                A = A * (A_limit / A_norm);
            end
            
            disp(norm(A));

            % 计算实际磁力矩
            M_m = cross(A, B);
        end

        
        function [detect_by_sensor_x, detect_by_sensor_z]...
                = SunSensor(obj, attitude, q)
            % 判断传感器是否检测到太阳
            % q: 太阳位置
            % attitude: 卫星姿态[roll, pitch, yaw](rad)
            
            % 地心惯性坐标系->本体坐标系
            R = eul2rotm(attitude, 'XYZ')';
            q = (R*q')';
            % 传感器视场(rad)
            sensor_field = obj.SensorField;
            % 传感器指向方向
            z = [0,0,1];
            x = [1,0,0];
            cos_half_field = cos(sensor_field / 2);
            % 检测逻辑
            detect_by_sensor_z = (q * z') > cos_half_field;
            detect_by_sensor_x = (q * x') > cos_half_field;
        end

        function in_shade = IsInShade(~, q, r)
            % 判断卫星是否在地影区
            q = q ./ norm(q);
            phi = acos(dot(q,r)/norm(r));
            in_shade = 0;
            if (phi > pi/2) && (norm(r)*sin(phi) < 6371)
                in_shade = 1;
            end
        end

        function e_t = TargetAttitude(~,q)
            % 反解姿态
            q = q / norm(q);
            roll = 0;
            yaw = asin(q(2));
            pitch = atan(-q(3)/q(1));
        
            e_t = [roll, pitch, yaw];
        end
   
        % 绘制卫星, 太阳能板, 传感器视场 PASS
        function obj = InitializeCubeSat(obj, shape_scale)
            % 初始化卫星显示对象
            length = obj.Shape(1) * shape_scale;
            width = obj.Shape(2) * shape_scale;
            height = obj.Shape(3) * shape_scale;
            
            % 存储原始顶点数据
            obj.OriginalVertices = [
                % 卫星体
                -length/2, -width/2, -height/2;
                 length/2, -width/2, -height/2;
                 length/2,  width/2, -height/2;
                -length/2,  width/2, -height/2;
                -length/2, -width/2,  height/2;
                 length/2, -width/2,  height/2;
                 length/2,  width/2,  height/2;
                -length/2,  width/2,  height/2;
                % 太阳能板
                 length/2, -width/2-length, -height/2;
                 length/2, -width/2-length, height/2;
                 length/2, width/2+length, -height/2;
                 length/2, width/2+length, height/2;
            ];
            
            % 面的定义
            faces = [
                % 卫星体
                1, 2, 3, 4;
                5, 6, 7, 8;
                1, 2, 6, 5;
                2, 3, 7, 6;
                3, 4, 8, 7;
                4, 1, 5, 8;
                % 太阳能板
                2, 6, 10, 9;
                3, 7, 12, 11
            ];
            
            % 创建卫星本体对象
            obj.CubeSatBody = patch('Vertices', obj.OriginalVertices, ...
                                  'Faces', faces, ...
                                  'FaceColor', '#9B9FB5', ...
                                  'EdgeColor', 'k');
            
            % 初始化传感器视场锥
            cone_angle = obj.SensorField;
            cone_height = length * 2;
            cone_radius = cone_height * tan(cone_angle / 2);
            num_points = 50;
            theta = linspace(0, 2*pi, num_points);
            
            % X轴视场锥
            obj.OriginalConeXVertex = [length/2, 0, 0];
            obj.OriginalConeXBase = [ones(num_points, 1) * cone_height + length/2, ...
                                   cone_radius * cos(theta)', ...
                                   cone_radius * sin(theta)'];
            
            % Z轴视场锥
            obj.OriginalConeZVertex = [0, 0, height/2];
            obj.OriginalConeZBase = [cone_radius * cos(theta)', ...
                                   cone_radius * sin(theta)', ...
                                   ones(num_points, 1) * cone_height + height/2];
            
            % 创建视场锥对象
            % 创建圆锥侧面的面片索引
            cone_faces = [];
            for i = 1:num_points-1
                cone_faces = [cone_faces; 1, i+1, i+2];
            end
            cone_faces = [cone_faces; 1, num_points+1, 2]; % 闭合最后一个面片
            
            obj.SensorConeX = patch('Vertices', [obj.OriginalConeXVertex; obj.OriginalConeXBase], ...
                                  'Faces', cone_faces, ...
                                  'FaceColor', '#808080', ...
                                  'EdgeColor', 'none', ...
                                  'FaceAlpha', 0.3);
                                  
            obj.SensorConeZ = patch('Vertices', [obj.OriginalConeZVertex; obj.OriginalConeZBase], ...
                                  'Faces', cone_faces, ...
                                  'FaceColor', '#808080', ...
                                  'EdgeColor', 'none', ...
                                  'FaceAlpha', 0.3);
        end

        function UpdateCubeSat(obj, time, re_scale)
            % 更新卫星位置和姿态
            dt = obj.Dt;
            i = floor(time/dt);
            
            % 获取当前位置和姿态
            position = obj.PositionList(i, :) .* re_scale/6371;
            attitude = obj.AttitudeList(i, :);
            status = obj.StatusList(i,:);
            
            % 计算旋转矩阵
            R = eul2rotm(attitude, 'XYZ');
            
            % 更新卫星本体
            rotated_vertices = (R * obj.OriginalVertices')';
            translated_vertices = rotated_vertices + repmat(position, size(rotated_vertices, 1), 1);
            set(obj.CubeSatBody, 'Vertices', translated_vertices);
            
            % 更新视场锥
            % X轴视场锥
            cone_x_vertices = [obj.OriginalConeXVertex; obj.OriginalConeXBase];
            rotated_cone_x = (R * cone_x_vertices')';
            translated_cone_x = rotated_cone_x + repmat(position, size(rotated_cone_x, 1), 1);
            set(obj.SensorConeX, 'Vertices', translated_cone_x);
            
            % Z轴视场锥
            cone_z_vertices = [obj.OriginalConeZVertex; obj.OriginalConeZBase];
            rotated_cone_z = (R * cone_z_vertices')';
            translated_cone_z = rotated_cone_z + repmat(position, size(rotated_cone_z, 1), 1);
            set(obj.SensorConeZ, 'Vertices', translated_cone_z);
            
            % 更新视场锥颜色
            detect_by_x = status(3);
            detect_by_z = status(4);
            
            if detect_by_x
                set(obj.SensorConeX, 'FaceColor', '#90ee90');
            else
                set(obj.SensorConeX, 'FaceColor', '#808080');
            end
            
            if detect_by_z
                set(obj.SensorConeZ, 'FaceColor', '#90ee90');
            else
                set(obj.SensorConeZ, 'FaceColor', '#808080');
            end
        end

        % 绘制轨道
        function DrawOrbit(obj, time, re_scale)
            dt = obj.Dt;
            n = floor(time/dt);
            position = obj.PositionList(n,:) .* re_scale/6371;
            plot3(position(:,1), position(:,2), position(:,3), ...
                '.', 'Color','#ADD8E6');
        end
    end
end