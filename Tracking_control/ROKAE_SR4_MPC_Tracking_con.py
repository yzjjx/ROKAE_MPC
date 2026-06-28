import time
import csv
import sys
import os
import mujoco
import mujoco.viewer
import numpy as np

# 1. 导入并初始化控制器
current_dir = os.path.dirname(os.path.abspath(__file__))
build_dir = os.path.join(current_dir, '../build')
build_release_dir = os.path.join(current_dir, '../build/Release') 
# 兼容 Windows MSVC 编译路径

sys.path.append(build_dir)
sys.path.append(build_release_dir)

try:
    # 替换为你的 ROKAE 编译模块
    import MPC_ROKAE
    print("成功导入 C++ 机器人 MPC 模块")
except ImportError as e:
    print(f"导入失败, 错误原因: {e}")
    sys.exit(1)

# 2. 加载机器人模型和数据 (请将路径替换为你真实的 ROKAE 机器人 xml 文件)
xml_path = "../xml/xMateSR4C.xml" 
if not os.path.exists(xml_path):
    print(f"警告: 未找到模型文件 {xml_path}，请修改为实际的机器人 XML 路径")
    sys.exit(1)

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 实例化 ROKAE MPC 控制器
mpc = MPC_ROKAE.RokaeMPC()

# 命令行输入期望关节位置
ref_input = input("输入关节期望位置，单位为度，用空格分隔")
q_ref_deg = np.array([float(x) for x in ref_input.split()],dtype=np.float64)

q_ref = np.deg2rad(q_ref_deg).astype(np.float64)

# 获取系统物理参数
dt = model.opt.timestep
num_q = model.nq  # 广义坐标数量 (关节数)
num_v = model.nv  # 广义速度数量
num_u = model.nu  # 执行器数量 (控制维度)

print(f"成功加载机器人模型。关节数(nq): {num_q}, 速度维数(nv): {num_v}, 控制轴数(nu): {num_u}")

# 可选：设置机器人的初始姿态（为每个关节的角度）
initial_degrees = [0.0, 30.0, 30.0, -0.0, 0.0, 0.0]
data.qpos[:] = np.deg2rad(initial_degrees)

# 初始化缓存列表
input_log = []   # 存放输入数据
output_log = []  # 存放输出数据

print("MuJoCo 机器人仿真启动...")

# 3. 启动仿真循环
print("MuJoCo 机器人仿真启动...")

# ==========================================
# 【新增】设置画面渲染帧率，限制在 60 FPS
# ==========================================
render_fps = 60
render_interval = 1.0 / render_fps
last_render_time = time.time()

with mujoco.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():
        step_start = time.time()

        # 1. 核心修改：机器人状态通常是全关节角度与全关节速度的拼接向量
        current_state = np.concatenate([data.qpos, data.qvel]).astype(np.float64)

        # 2. 计算最优控制力矩
        mpc_start = time.time()
        u_opt = mpc.compute_control(current_state,q_ref)
        mpc_time = time.time() - mpc_start
        
        # 警告机制：如果 MPC 计算时间超过了物理步长，必然会引起卡顿！
        if mpc_time > dt:
            print(f"[警告] MPC 计算超时: {mpc_time*1000:.1f} ms (允许阈值: {dt*1000:.1f} ms)")

        # 3. 将控制量施加到机器人所有执行器上
        data.ctrl[:] = u_opt

        # 4. 采集数据
        input_log.append([data.time] + data.ctrl.tolist())
        output_log.append([data.time] + data.qpos.tolist() + data.qvel.tolist())

        # 5. 推进物理引擎
        mujoco.mj_step(model, data)

        # ==========================================
        # 6. 【核心修复】渲染解耦：只在时间间隔足够时才刷新画面
        # ==========================================
        current_time = time.time()
        if (current_time - last_render_time) >= render_interval:
            viewer.sync()
            last_render_time = current_time

        # 7. 严格锁帧，保证仿真速度与现实时间 1:1 同步
        time_until_next_step = dt - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

# 4. 仿真结束后保存 CSV
print("\n正在保存仿真数据...")

output_dir = os.path.abspath(os.path.join(current_dir, '../python_output'))
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 动态生成控制输入的表头，例如: ['time', 'ctrl_0', 'ctrl_1', ...]
ctrl_headers = ['time'] + [f'ctrl_{i}' for i in range(num_u)]
with open(os.path.join(output_dir, 'control_inputs.csv'), 'w', newline='', encoding='utf-8') as f:
    writer = csv.writer(f)
    writer.writerow(ctrl_headers)
    writer.writerows(input_log)

# 动态生成系统状态输出的表头，例如: ['time', 'q_0', ..., 'v_0', ...]
state_headers = ['time'] + [f'q_{i}' for i in range(num_q)] + [f'v_{i}' for i in range(num_v)]
with open(os.path.join(output_dir, 'sys_outputs.csv'), 'w', newline='', encoding='utf-8') as f:
    writer = csv.writer(f)
    writer.writerow(state_headers)
    writer.writerows(output_log)

print(f"数据保存成功！文件保存在: {output_dir}")