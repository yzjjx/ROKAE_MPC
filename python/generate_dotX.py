# 该代码用来生成矩阵dotX，详细见README文档
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
# 创建力矩输入 tau
ctau = casadi.SX.sym('tau', ndq, 1)

# 调用 Pinocchio-CasADi底层算法进行符号计算
# 符号推导惯性矩阵 M(q)
cpin.crba(cmodel, cdata, cq)

# crba 只算上三角，利用 CasADi 的函数将其对称化
M_upper = casadi.triu(cdata.M, True)          # 包含对角线的上三角
M_strict_upper = casadi.triu(cdata.M, False)  # 不包含对角线的严格上三角
M_sym = M_upper + M_strict_upper.T

# 符号推导科里奥利力与离心力矩阵 C(q, dq)
cpin.computeCoriolisMatrix(cmodel, cdata, cq, cdq)
C_sym = cdata.C

# 符号推导重力矢量 G(q)
cpin.computeGeneralizedGravity(cmodel, cdata, cq)
G_sym = cdata.g

# # 将这些符号表达式打包成一个CasADi函数，函数名称为compute_dynamics
# # 输入：[q, dq]   输出：[M, C, G]
# dyn_func = casadi.Function('compute_dynamics', 
#                            [cq, cdq], [M_sym, C_sym, G_sym], 
#                            ['q', 'dq'], ['M', 'C', 'G'])

# # 自动生成纯 C 语言代码
# # 生成对应的 .h 头文件
# opts = dict(main=False, mex=False, with_header=True)
# # 生成C代码
# dyn_func.generate('robot_dynamics.c', opts)
# M本身就是对称矩阵
ddq_sym = casadi.solve(M_sym, ctau - C_sym @ cdq - G_sym)
dotX_sym = casadi.vertcat(cdq, ddq_sym)

# 打包成 CasADi 函数
dotX_func = casadi.Function(
    'compute_dotX',
    [cq, cdq, ctau],
    [dotX_sym],
    ['q', 'dq', 'tau'],
    ['dotX']
)

# 生成 C 代码
opts = dict(main=False, mex=True, with_header=True)
dotX_func.generate('robot_dotX.c', opts)

print("成功生成 robot_dotX.c 和 robot_dotX.h")