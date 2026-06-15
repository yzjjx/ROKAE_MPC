%% 清屏
clear; close all; clc;

%% 添加 mex 文件路径
addpath('../python');

% 如果还没编译，可进入 ../python 后编译：
% cd ../python
% mex robot_AB.c -largeArrayDims
% mex robot_dotX.c -largeArrayDims
% mex robot_dynamics_G.c -largeArrayDims
% cd ../matlab

%% 机器人维度
nq = 6;
ndq = 6;
n = nq + ndq;
p = ndq;

%% MPC 参数
Ts = 0.01;

Q = diag([100 100 100 100 100 100 ...
          1   1   1   1   1   1]);

F = Q;
R = 0.01 * eye(p);

k_steps = 3000;
N = 60;

X_K = zeros(n, k_steps+1);
U_K = zeros(p, k_steps);        % 实际输入力矩 = MPC反馈 + 重力补偿
U_MPC_K = zeros(p, k_steps);    % 仅MPC反馈项
G_K = zeros(p, k_steps);        % 重力补偿项

X_K(:,1) = [
     0.10;
    -0.05;
     0.08;
    -0.03;
     0.05;
    -0.08;
     0;
     0;
     0;
     0;
     0;
     0
];

%% LTV-MPC 循环
for k = 1:k_steps

    X_current = X_K(:, k);

    q_current  = X_current(1:nq);
    dq_current = X_current(nq+1:n);

    if k == 1
        tau_linearize = zeros(p,1);
    else
        tau_linearize = U_K(:,k-1);
    end

    % 线性化
    [Ac, Bc] = robot_AB(q_current, dq_current, tau_linearize);

    Ac = full(Ac);
    Bc = full(Bc);

    A_k = eye(n) + Ac * Ts;
    B_k = Bc * Ts;

    % MPC 矩阵
    [E, H] = MPC_Matrices(A_k, B_k, Q, R, F, N);

    % MPC反馈控制项
    % full函数用来将稀疏矩阵或者某些特殊矩阵转换为普通的密集矩阵（即MATLAB可以使用的矩阵）
    tau_mpc = Prediction(X_current, E, H, N, p);
    tau_mpc = full(tau_mpc);

    % 重力补偿项 G(q)
    G_current = robot_dynamics_G(q_current);
    G_current = full(G_current);

    % 总控制力矩
    U_MPC_K(:,k) = tau_mpc;
    G_K(:,k) = G_current;
    U_K(:,k) = tau_mpc + G_current;

    % 非线性系统更新
    dotX_current = robot_dotX(q_current, dq_current, U_K(:,k));
    dotX_current = full(dotX_current);

    X_K(:, k+1) = X_current + dotX_current * Ts;
end

%% 数据可视化
t = 0:k_steps;

figure('Position', [100, 100, 900, 900]);

subplot(4,1,1);
plot(t, X_K(1:6, :)', 'LineWidth', 1.2);
grid on;
title('关节位置 q');
xlabel('控制步数 k');
ylabel('q / rad');
legend('q_1','q_2','q_3','q_4','q_5','q_6');

subplot(4,1,2);
plot(t, X_K(7:12, :)', 'LineWidth', 1.2);
grid on;
title('关节速度 dq');
xlabel('控制步数 k');
ylabel('dq / rad/s');
legend('dq_1','dq_2','dq_3','dq_4','dq_5','dq_6');

subplot(4,1,3);
plot(1:k_steps, U_MPC_K', 'LineWidth', 1.2);
grid on;
title('MPC反馈力矩 \tau_{mpc}');
xlabel('控制步数 k');
ylabel('\tau_{mpc} / Nm');
legend('\tau_{mpc,1}','\tau_{mpc,2}','\tau_{mpc,3}', ...
       '\tau_{mpc,4}','\tau_{mpc,5}','\tau_{mpc,6}');

subplot(4,1,4);
plot(1:k_steps, U_K', 'LineWidth', 1.2);
grid on;
title('实际控制力矩 \tau = \tau_{mpc} + G(q)');
xlabel('控制步数 k');
ylabel('tau / Nm');
legend('\tau_1','\tau_2','\tau_3','\tau_4','\tau_5','\tau_6');