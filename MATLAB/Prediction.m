function u_k = Prediction(x_k, E, H, N, p)

    % 强制 Hessian 对称，避免 quadprog 警告
    H = (H + H') / 2;

    % 一次项
    f = 2 * E' * x_k;

    % 输入约束
    umin = -20;
    umax = 20;

    lb = umin * ones(N*p, 1);
    ub = umax * ones(N*p, 1);

    % 初值
    U0 = zeros(N*p, 1);

    % quadprog 设置
    options = optimoptions('quadprog', ...
        'Display', 'off', ...
        'Algorithm', 'interior-point-convex');

    % 求解 QP
    [U_k, ~, exitflag] = quadprog(2*H, f, [], [], [], [], lb, ub, U0, options);

    % 如果求解失败，给一个安全控制量
    if exitflag <= 0 || any(~isfinite(U_k))
        warning('quadprog 求解失败，控制输入置为 0');
        u_k = zeros(p,1);
    else
        u_k = U_k(1:p);
    end
end