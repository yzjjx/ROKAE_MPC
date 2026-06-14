import pinocchio as pin
import numpy as np

# URDF
urdf_filename = "../urdf/xMateSR4C_gen.urdf"

# 构建 Pinocchio 模型
model = pin.buildModelFromUrdf(urdf_filename)
data = model.createData()

# 获取自由度
nq = model.nq
nv = model.nv

print("nq =", nq)
print("nv =", nv)

# 4. 输入 q, dq, ddq
# 关节位置 q
q = np.array([0.0, 0.2, -0.3, 0.1, 0.5, -0.2])

# 关节速度 dq
dq = np.array([0.1, 0.0, -0.2, 0.0, 0.1, -0.1])

# 关节加速度 ddq
ddq = np.array([0.0, 0.5, 0.0, -0.2, 0.0, 0.1])

# RNEA 逆动力学计算
tau = pin.rnea(model, data, q, dq, ddq)

# 6. 输出 tau
print("RNEA 输出关节力矩 tau：")
print(tau)
