#include <iostream>
#include <string>

// Pinocchio 相关头文件
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/crba.hpp"       // 包含计算 M(q) 的 crba 算法
#include "pinocchio/algorithm/compute-all-terms.hpp"  // 包含计算 C(q, v) 的方法
#include "pinocchio/algorithm/rnea.hpp"       // 包含计算 G(q) 的方法
#include "pinocchio/algorithm/joint-configuration.hpp" // 用于生成随机关节配置

int main(int argc, char ** argv)
{
    // 1. 指定 URDF 文件路径 (可以通过命令行参数传入，否则使用默认路径)
    std::string urdf_filename = (argc > 1) ? argv[1] : "/path/to/your/robot.urdf";

    // 2. 建立机器人模型
    pinocchio::Model model;
    try {
        // 注意：这里默认建立的是固定基座(Fixed-base)的机器人模型。
        // 如果是浮动基座(如四足机器人/人形机器人)，需改为：
        // pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), model);
        pinocchio::urdf::buildModel(urdf_filename, model);
    } catch (const std::invalid_argument& e) {
        std::cerr << "加载 URDF 失败: " << e.what() << std::endl;
        std::cerr << "请检查路径是否正确: " << urdf_filename << std::endl;
        return -1;
    }

    std::cout << "成功加载模型!" << std::endl;
    std::cout << "模型自由度 (nv): " << model.nv << std::endl;
    std::cout << "位置变量维度 (nq): " << model.nq << std::endl;

    // 3. 初始化与模型匹配的数据结构 (Data 包含了计算过程中所有的中间变量和结果)
    pinocchio::Data data(model);

    // 4. 设置关节位置 q 和 关节速度 v (这里我们生成合法的随机状态)
    Eigen::VectorXd q = pinocchio::randomConfiguration(model);
    Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);

    // ==========================================
    // 5. 计算动力学矩阵
    // ==========================================

    // A. 计算惯性矩阵 M(q) 
    // 使用复合刚体算法 (CRBA)。
    // 【关键细节】：为了提高效率，Pinocchio的 crba() 默认只计算并填充了 M 的上三角部分。
    pinocchio::crba(model, data, q);
    // 将上三角复制到下三角，使其成为完整的对称矩阵
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

    // B. 计算科里奥利力与离心力矩阵 C(q, v)
    pinocchio::computeCoriolisMatrix(model, data, q, v);

    // C. 计算广义重力矢量 G(q)
    pinocchio::computeGeneralizedGravity(model, data, q);

    // ==========================================
    // 6. 打印输出结果
    // ==========================================
    std::cout << "\n--- 当前关节状态 ---" << std::endl;
    std::cout << "q (位置): " << q.transpose() << std::endl;
    std::cout << "v (速度): " << v.transpose() << std::endl;

    std::cout << "\n--- 惯性矩阵 M(q) [" << model.nv << "x" << model.nv << "] ---" << std::endl;
    std::cout << data.M << std::endl;

    std::cout << "\n--- 科里奥利力与离心力矩阵 C(q, v) [" << model.nv << "x" << model.nv << "] ---" << std::endl;
    std::cout << data.C << std::endl;

    std::cout << "\n--- 重力矢量 G(q) [" << model.nv << "x1] ---" << std::endl;
    std::cout << data.g.transpose() << std::endl;

    return 0;
}