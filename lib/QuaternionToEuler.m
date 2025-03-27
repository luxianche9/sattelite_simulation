function attitude = QuaternionToEuler(q)
    % 转化四元数到欧拉姿态角
    % q: 四元数[w, x, y, z]
    % attitude: 欧拉姿态角[roll, pitch, yaw](rad)

    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    % Roll (X-axis rotation)
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x^2 + y^2);
    roll = atan2(sinr_cosp, cosr_cosp);

    % Pitch (Y-axis rotation)
    sinp = 2 * (w * y - z * x);
    % Handle numerical instability
    if abs(sinp) >= 1
        pitch = sign(sinp) * pi / 2;
    else
        pitch = asin(sinp);
    end

    % Yaw (Z-axis rotation)
    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y^2 + z^2);
    yaw = atan2(siny_cosp, cosy_cosp);
    
    attitude = [roll, pitch, yaw];
end
