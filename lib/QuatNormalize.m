function q_norm = QuatNormalize(q)
    % 单位化四元数
    % q: 四元数[w, x, y, z]
    q_norm = q / norm(q);
end