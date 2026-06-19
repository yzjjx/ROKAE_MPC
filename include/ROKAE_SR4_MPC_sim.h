#ifndef CARTPOLE_MPC_H
#define CARTPOLE_MPC_H

#include <Eigen/Dense>
// 引入 CasADi 生成的 C 头文件
extern "C" {
    #include "robot_AB.h"
}

// 声明生成的 CasADi 重力补偿函数 (防止 C++ 找不到 C 语言的函数名)
extern "C" {
    // 假设你使用默认的 casadi_int，通常是 long long
    // 如果编译报错找不到 casadi_int，请将下面的 casadi_int 替换为 long long
    int compute_dynamics_G(const double** arg, double** res, long long* iw, double* w, int mem);
    int compute_dynamics_G_work(long long *sz_arg, long long* sz_res, long long *sz_iw, long long *sz_w);
}


// private代表私有属性，即只能在对应的cpp文件被使用和修改，外部不能被使用和修改
// public为外部公开结构，外部代码只能通过调用这些接口让这个class工作
// CartpoleMPC();：为“构造函数”，名字必须和类名一模一样。它的作用是：当别人新建这个类的对象
// 时，负责把 Q, R 这些矩阵初始化好
class RokaeMPC {
private:
    double Ts;
    int n, p, N, n_dof;
    Eigen::MatrixXd Q, R, F;
    Eigen::VectorXd last_u;
public:
    RokaeMPC();
    // current_state 包含 [q, dq]，共 2*n_dof 维
    Eigen::VectorXd compute_control(const Eigen::VectorXd& current_state);
};

#endif