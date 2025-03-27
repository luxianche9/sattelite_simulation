function q = EulerToQuaternion(attitude)
    % 转化欧拉姿态角到四元数
    % attitude: 欧拉姿态角[roll, pitch, yaw](rad)
    % q: 四元数[w, x, y, z]

    roll = attitude(1);
    pitch = attitude(2);
    yaw = attitude(3);
    cr = cos(roll / 2);
    sr = sin(roll / 2);
    cp = cos(pitch / 2);
    sp = sin(pitch / 2);
    cy = cos(yaw / 2);
    sy = sin(yaw / 2);

    % Compute quaternion components
    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;

    % Return the quaternion as [w, x, y, z]
    q = [w, x, y, z];
end