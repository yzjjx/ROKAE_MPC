#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

extern "C" {
#include "../include/robot_dynamics.h"
}

int main()
{
    // 加载 URDF,因为windows里面是在release文件夹内运行，因此需要提前两个文件夹
    std::string urdf_path = "../../urdf/xMateSR4C_gen.urdf";

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::Data data(model);

    int nq = model.nq;
    int nv = model.nv;

    // 输入 q, dq, ddq
    Eigen::VectorXd q(nq);
    Eigen::VectorXd dq(nv);
    Eigen::VectorXd ddq(nv);

    q<<0.0, 0.2, -0.3, 0.1, 0.5, -0.2;
    dq<<0.1, 0.0, -0.2, 0.0, 0.1, -0.1;
    ddq<<0.0, 0.5, 0.0, -0.2, 0.0, 0.1;

    // 调用 robot_dynamics.c
    // 输入: q, dq
    // 输出: M, C, G

    std::vector<double> q_in(nq);
    std::vector<double> dq_in(nv);

    for (int i = 0; i < nq; ++i)
        q_in[i] = q[i];

    for (int i = 0; i < nv; ++i)
        dq_in[i] = dq[i];

    std::vector<double> M_out(nv * nv);
    std::vector<double> C_out(nv * nv);
    std::vector<double> G_out(nv);

    const double* arg[2];
    double* res[3];

    arg[0] = q_in.data();
    arg[1] = dq_in.data();

    res[0] = M_out.data();
    res[1] = C_out.data();
    res[2] = G_out.data();

    long long sz_arg, sz_res, sz_iw, sz_w;
    compute_dynamics_work(&sz_arg, &sz_res, &sz_iw, &sz_w);

    std::vector<long long> iw(sz_iw);
    std::vector<double> w(sz_w);

    int flag = compute_dynamics(arg, res, iw.data(), w.data(), 0);

    if (flag != 0)
    {
        std::cerr << "compute_dynamics failed, flag = " << flag << std::endl;
        return -1;
    }

    // CasADi 输出是列主序，Eigen 默认也是列主序
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> M(
        M_out.data(), nv, nv);

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> C(
        C_out.data(), nv, nv);

    Eigen::Map<Eigen::VectorXd> G(
        G_out.data(), nv);

    // 用 M, C, G 计算 tau
    // tau = M * ddq + C * dq + G
    Eigen::VectorXd tau_from_c = M * ddq + C * dq + G;


    // 输出
    std::cout << "\nM from robot_dynamics.c:\n" << M << std::endl;
    std::cout << "\nC from robot_dynamics.c:\n" << C << std::endl;
    std::cout << "\nG from robot_dynamics.c:\n" << G.transpose() << std::endl;

    std::cout << "\ntau_from_c =\n" << tau_from_c.transpose() << std::endl;
    return 0;
}