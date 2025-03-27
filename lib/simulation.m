function [r_list, v_list, e_list, w_list, W_list]...
       = simulation(r0, v0, e0, w0, W0, time, dt)
    % 微分方程求解辅助函数: 处理多变量
    % 欧拉角转换为四元数
    % r 位置[x,y,z](km)
    % v 速度[v_x, v_y, v_z](km/s)
    % e 姿态[roll, pitch, yaw](rad)
    % q 四元数[w, x, y, z]
    % w 卫星角速度[w_x, w_y, w_z](rad/s)
    % W 飞轮角速度(rad/s)

    % 欧拉角转换成四元数
    q0 = EulerToQuaternion(e0);
    
    y0 = [r0,v0,q0,w0,W0];
    
    % 欧拉矫正预测法求解微分方程
    y = EulerPredictCorrect(time, dt, y0, @f);
    
    r_list = y(:,1:3);
    v_list = y(:,4:6);

    % 四元数转换为欧拉角
    q_list = y(:,7:10);
    length_of_list = size(q_list,1);
    e_list = zeros(length_of_list, 3);
    for i = 1: length_of_list
        q = QuatNormalize(q_list(i,:));
        e_list(i,:) = QuaternionToEuler(q);
    end

    w_list = y(:,11:13);
    W_list = y(:,14:16);
end