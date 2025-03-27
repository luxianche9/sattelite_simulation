% 输入地球表面某点的坐标 (单位：米)
x = 6371e3; % 地球半径，假设在赤道
y = 0;
z = 0;

% 计算该位置的磁场
B = dipole_magnetic_field(x, y, z);

% 输出磁场的各分量
disp('Magnetic field components:');
disp(B);




function B = dipole_magnetic_field(x, y, z)
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

