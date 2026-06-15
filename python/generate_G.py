# 该代码用来生成后续计算所需要的机器人G矩阵，主要用来进行重力补偿
import casadi
import pinocchio as pin
# 导入Pinocchio的CasADi接口
import pinocchio.casadi as cpin

# 加载urdf文件
urdf_filename = "../urdf/xMateSR4C_gen.urdf"
model = pin.buildModelFromUrdf(urdf_filename)

# 转换为CasADi模型
cmodel = cpin.Model(model)
cdata = cmodel.createData()

# 获取urdf的机器人自由度
nq = cmodel.nq
ndq = cmodel.nv #获取速度变量的维度

# 创建 CasADi 符号变量 (SX: 标量表达式，适合生成高效 C 代码)
# SX：适合标量表达式展开，生成 C 代码通常较高效
# MX：适合复杂图结构、较大规模优化问题
# DM：数值矩阵，不是符号变量
cq = casadi.SX.sym('q', nq, 1)# 创建一个名为q的符号列向量，维度为nq*1
cdq = casadi.SX.sym('dq', ndq, 1)

# 符号推导重力矢量 G(q)
cpin.computeGeneralizedGravity(cmodel, cdata, cq)
G_sym = cdata.g

# 将这些符号表达式打包成一个CasADi函数，函数名称为compute_dynamics
# 输入：[q, dq]   输出：[M, C, G]
dyn_func = casadi.Function('compute_dynamics_G', 
                           [cq], [G_sym], 
                           ['q'], ['G'])

# 自动生成纯 C 语言代码
# 生成对应的 .h 头文件
opts = dict(main=False, mex=True, with_header=True)
# 生成C代码
dyn_func.generate('robot_dynamics_G.c', opts)

print("成功生成 robot_dynamics_G.c 和 robot_dynamics_G.h")