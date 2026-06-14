%% 清屏
clear; close all; clc;

%% 添加 mex 文件路径
addpath('../python');

% cd ../python
% 
% mex robot_AB.c -largeArrayDims
% mex robot_dotX.c -largeArrayDims

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
U_K = zeros(p, k_steps);

X_K(:,1) = [
     0.10;   % q1
    -0.05;   % q2
     0.08;   % q3
    -0.03;   % q4
     0.05;   % q5
    -0.08;   % q6
     0;      % dq1
     0;      % dq2
     0;      % dq3
     0;      % dq4
     0;      % dq5
     0       % dq6
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

    % 直接调用 robot_AB.mexw64
    [Ac, Bc] = robot_AB(q_current, dq_current, tau_linearize);

    Ac = full(Ac);
    Bc = full(Bc);

    A_k = eye(n) + Ac * Ts;
    B_k = Bc * Ts;

    [E, H] = MPC_Matrices(A_k, B_k, Q, R, F, N);

    U_K(:,k) = Prediction(X_current, E, H, N, p);

    % 直接调用 robot_dotX.mexw64
    dotX_current = robot_dotX(q_current, dq_current, U_K(:,k));
    dotX_current = full(dotX_current);

    X_K(:, k+1) = X_current + dotX_current * Ts;
end
%% 五：数据可视化
t = 0:k_steps;

figure('Position', [100, 100, 900, 800]);

subplot(3,1,1);
plot(t, X_K(1:6, :)', 'LineWidth', 1.2);
grid on;
title('关节位置 q');
xlabel('控制步数 k');
ylabel('q / rad');
legend('q_1','q_2','q_3','q_4','q_5','q_6');

subplot(3,1,2);
plot(t, X_K(7:12, :)', 'LineWidth', 1.2);
grid on;
title('关节速度 dq');
xlabel('控制步数 k');
ylabel('dq / rad/s');
legend('dq_1','dq_2','dq_3','dq_4','dq_5','dq_6');

subplot(3,1,3);
plot(1:k_steps, U_K', 'LineWidth', 1.2);
grid on;
title('控制力矩 tau');
xlabel('控制步数 k');
ylabel('tau / Nm');
legend('\tau_1','\tau_2','\tau_3','\tau_4','\tau_5','\tau_6');