#include "ROKAE_SR4_MPC_sim.h"
#include <cmath>
#include <algorithm>
#include "MPC_Matrices.h"
#include "Prediction.h"

// 实现构造函数，注意前缀 CartpoleMPC::
RokaeMPC::RokaeMPC() {
    Ts = 0.02;
    n_dof = 6;          
    n = 2 * n_dof;       // 状态维度：12
    p = n_dof;           // 输入维度：6
    N = 20;              // 预测步长

    // 重新初始化 Q, R, F 矩阵尺寸
    Eigen::VectorXd q_diag(n);
    q_diag.head(n_dof).setConstant(100.0); // 前 6 项: 关节角度误差惩罚极为严厉 (100)
    q_diag.tail(n_dof).setConstant(1.0);   // 后 6 项: 关节速度误差惩罚较轻 (1)

    Q = q_diag.asDiagonal();
    F = Q;
    R = Eigen::MatrixXd::Identity(p, p) * 0.01;
    
    last_u = Eigen::VectorXd::Zero(p);
}

// 输入当前状态，输出推力u
Eigen::VectorXd RokaeMPC::compute_control(const Eigen::VectorXd& current_state) {
    Eigen::VectorXd q = current_state.head(n_dof);
    Eigen::VectorXd dq = current_state.tail(n_dof);
    Eigen::VectorXd tau = last_u; // 围绕上一步的控制输入线性化

    // 获取 CasADi 函数需要的内存工作区大小
    casadi_int sz_arg, sz_res, sz_iw, sz_w;
    compute_AB_work(&sz_arg, &sz_res, &sz_iw, &sz_w);

    // 分配指针数组和工作内存
    std::vector<const double*> arg(sz_arg, nullptr);
    std::vector<double*> res(sz_res, nullptr);
    std::vector<casadi_int> iw(sz_iw, 0);
    std::vector<double> w(sz_w, 0.0);

    // 3. 绑定输入指针 (对应 Python 中的 ['q', 'dq', 'tau'])
    arg[0] = q.data();
    arg[1] = dq.data();
    arg[2] = tau.data();

    // 4. 准备输出数据的容器
    std::vector<double> A_data(n * n, 0.0);
    std::vector<double> B_data(n * p, 0.0);
    
    // 绑定输出指针 (对应 Python 中的 ['A', 'B'])
    res[0] = A_data.data();
    res[1] = B_data.data();

    // 5. 执行 CasADi 生成的 C 函数
    // 第五个参数是 mem 标识符，通常传 0
    compute_AB(arg.data(), res.data(), iw.data(), w.data(), 0);

    // 6. 利用 Eigen::Map 将 1D 数组映射为 Eigen 矩阵 (列主序，完美匹配)
    Eigen::Map<Eigen::MatrixXd> Ac(A_data.data(), n, n);
    Eigen::Map<Eigen::MatrixXd> Bc(B_data.data(), n, p);

    static bool first_print = true;
    if(first_print) {
        std::cout << "\n===== [CasADi] Ac (first 5x5 block) =====\n";
        std::cout << Ac.block(0,0,5,5) << std::endl;
        std::cout << "\n===== [CasADi] Bc (first 5x5 block) =====\n";
        std::cout << Bc.block(0,0,5,5) << std::endl;
        first_print = false;
    }

    // 7. 连续系统离散化 (欧拉法或精确离散化，这里保留你的欧拉法)
    Eigen::MatrixXd A_k = Eigen::MatrixXd::Identity(n, n) + Ac * Ts;
    Eigen::MatrixXd B_k = Bc * Ts;

    // 8. 调用 MPC 求解器
    MPC_Matrices result = MPC_cal(A_k, B_k, Q, R, F, N);
    
    // U_K 现在的维度应该是 p * N
    Eigen::VectorXd U_K = Prediction(current_state, result.E, result.H, N, p);
    
    // 9. 提取第一步控制量并限幅
    Eigen::VectorXd u_feedback = U_K.head(p);
    
    // // 加入重力补偿矩阵
    // long long sz_arg_G, sz_res_G, sz_iw_G, sz_w_G;
    // compute_dynamics_G_work(&sz_arg_G, &sz_res_G, &sz_iw_G, &sz_w_G);

    // std::vector<const double*> arg_G(sz_arg_G, nullptr);
    // std::vector<double*> res_G(sz_res_G, nullptr);
    // std::vector<long long> iw_G(sz_iw_G, 0);
    // std::vector<double> w_G(sz_w_G, 0.0);

    // // 绑定输入：q
    // arg_G[0] = q.data();

    // // 绑定输出：G矩阵 (维度为 6x1)
    // std::vector<double> G_data(n_dof, 0.0);
    // res_G[0] = G_data.data();

    // // 执行计算
    // compute_dynamics_G(arg_G.data(), res_G.data(), iw_G.data(), w_G.data(), 0);

    // // 映射回 Eigen 向量
    // Eigen::Map<Eigen::VectorXd> G_vec(G_data.data(), p);

    // 实际控制力矩
    Eigen::VectorXd actual_u = u_feedback;// + G_vec;

    // 对每个关节的力矩进行限幅 (假设最大扭矩限制为 50 Nm)
    for(int i = 0; i < p; ++i) {
        actual_u(i) = std::max(-300.0, std::min(300.0, actual_u(i)));
    }
    
    last_u = actual_u;
    return actual_u;

}