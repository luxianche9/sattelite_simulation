clear;
clc;
% [roll, pitch, yaw]
attitude = [0.1, 0.2, 0.3];

R_roll = [
    1, 0, 0;
    0, cos(attitude(1)), -sin(attitude(1));
    0, sin(attitude(1)), cos(attitude(1))
];
R_pitch = [
    cos(attitude(2)), 0, sin(attitude(2));
    0, 1, 0;
    -sin(attitude(2)), 0, cos(attitude(2))
];
R_yaw = [
    cos(attitude(3)), -sin(attitude(3)), 0;
    sin(attitude(3)), cos(attitude(3)), 0;
    0, 0, 1
];

R = R_roll * R_pitch * R_yaw;
inv_R = R_yaw' * R_pitch' * R_roll';
I = R * inv_R;
disp(R);
disp(inv_R);
disp(I);

% 使用
R2 = eul2rotm(attitude, 'XYZ');
inv_R2 = R2';
I2 = R2 * inv_R2;
disp(R2);
disp(inv_R2);
disp(I2);