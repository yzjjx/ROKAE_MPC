# 该代码用来生成后续计算所需要的机器人M矩阵、C矩阵和G矩阵
import casadi
import pinocchio as pin
import pinocchio.casadi as cpin

# 加载urdf文件
urdf_filename = "../urdf/xMateSR4C_gen.urdf"
model = pin.buildModelFromUrdf(urdf_filename)

# 转换为CasADi模型
cmodel = cpin.Model(model)
cdata = cmodel.createData()

# 获取urdf的机器人自由度
nq = cmodel.nq
nv = cmodel.nv

# 创建 CasADi 符号变量 (SX: 标量表达式，适合生成高效 C 代码)
cq = casadi.SX.sym('q', nq, 1)
cv = casadi.SX.sym('dq', nv, 1)

# 调用 Pinocchio-CasADi底层算法进行符号计算
# 符号推导惯性矩阵 M(q)
cpin.crba(cmodel, cdata, cq)

# crba 只算上三角，利用 CasADi 的函数将其对称化
M_upper = casadi.triu(cdata.M, True)          # 包含对角线的上三角
M_strict_upper = casadi.triu(cdata.M, False)  # 不包含对角线的严格上三角
M_sym = M_upper + M_strict_upper.T

# 符号推导科里奥利力与离心力矩阵 C(q, v)
cpin.computeCoriolisMatrix(cmodel, cdata, cq, cv)
C_sym = cdata.C

# 符号推导重力矢量 G(q)
cpin.computeGeneralizedGravity(cmodel, cdata, cq)
G_sym = cdata.g

# 将这些符号表达式打包成一个CasADi函数
# 输入：[q, dq]   输出：[M, C, G]
dyn_func = casadi.Function('compute_dynamics', 
                           [cq, cv], [M_sym, C_sym, G_sym], 
                           ['q', 'dq'], ['M', 'C', 'G'])

# 你甚至可以在 Python 里测试一下这个函数
# import numpy as np
# res = dyn_func(q=np.zeros(nq), v=np.zeros(nv))
# print("测试计算得到的重力 G:\n", res['G'])

# 自动生成纯 C 语言代码
opts = dict(main=False, mex=False, with_header=True) # 生成对应的 .h 头文件
dyn_func.generate('robot_dynamics.c', opts)

print("成功生成 robot_dynamics.c 和 robot_dynamics.h")