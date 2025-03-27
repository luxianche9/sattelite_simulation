clear;

q = [0.758342066799373	-0.598178021973435	-0.259037379060250];
fprintf("q:[%d, %d, %d]\n",...
            q(1), q(2), q(3));

attitude = TargetAttitude(q);
fprintf("roll:%d pitch:%d yaw:%d\n",...
        attitude(1), attitude(2), attitude(3));

x = [1,0,0];
R = eul2rotm(attitude, 'XYZ');
x_r = (R*x')';
fprintf("x:[%d, %d, %d]\n",...
         x_r(1), x_r(2), x_r(3));

e = x_r - q;
fprintf("error:[%d, %d, %d]\n",...
         e(1), e(2), e(3));

function attitude = TargetAttitude(q)
    q = q / norm(q);
    roll = 0;
    yaw = asin(q(2));
    pitch = atan(-q(3)/q(1));

    attitude = [roll, pitch, yaw];
end