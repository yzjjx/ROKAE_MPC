import time
import sys
import os
import mujoco
import numpy as np

# 环境与依赖初始化,主要是找到.so文件的位置
current_dir = os.path.dirname(os.path.abspath(__file__))
build_dir = os.path.join(current_dir, '../build')
build_release_dir = os.path.join(current_dir, '../build/Release') 

sys.path.append(build_dir)
sys.path.append(build_release_dir)

# try函数可以在错误的时候直接跳转到expect里面去处理，如果try里面出现了ImportError(导入错误)，就将错误信息抓取下来赋值给e
try:
    import MPC_ROKAE
except ImportError as e:
    print(f"导入失败, 错误原因: {e}")
    sys.exit(1)

xml_path = "../xml/xMateSR4C.xml" 
if not os.path.exists(xml_path):
    print(f"错误: 未找到模型文件 {xml_path}")
    sys.exit(1)

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 实例化控制器
mpc = MPC_ROKAE.RokaeMPC()

# 初始化机器人姿态
initial_degrees = [0.0, 10.0, 10.0, -0.0, 0.0, 0.0]
data.qpos[:] = np.deg2rad(initial_degrees)

# 性能测试参数设置
WARMUP_STEPS = 50       # 预热步数（不计入统计，用于让CPU缓存和C++动态库热身）
TEST_STEPS = 2000       # 实际测试的步数
dt = model.opt.timestep # 设置dt为模型时间，可以在xml文件里面修改

print(f"---  MPC 性能测试 ---")
print(f"物理步长 (dt): {dt * 1000:.2f} ms")
print(f"预热步数: {WARMUP_STEPS}, 测试步数: {TEST_STEPS}")

# 预热阶段
# 先让控制器空跑一段时间，让数据进入CPU缓存
# for循环中加入横线（_）表示单纯让这一段代码运行50次
for _ in range(WARMUP_STEPS):
    # 获取机器人状态
    current_state = np.concatenate([data.qpos, data.qvel]).astype(np.float64)
    u_opt = mpc.compute_control(current_state)
    data.ctrl[:] = u_opt
    mujoco.mj_step(model, data)

# 正式测试阶段
# 初始化时间数组
execution_times = []

# 测试2000次
for _ in range(TEST_STEPS):
    current_state = np.concatenate([data.qpos, data.qvel]).astype(np.float64)
    
    # 使用 perf_counter 提供最高精度的纳秒级计时
    start_time = time.perf_counter() 
    u_opt = mpc.compute_control(current_state)
    end_time = time.perf_counter()
    
    # 记录耗时 (转换为毫秒)
    step_time_ms = (end_time - start_time) * 1000.0
    execution_times.append(step_time_ms)
    
    # 继续推进物理引擎以提供动态变化的 state 给下一步 MPC
    data.ctrl[:] = u_opt
    mujoco.mj_step(model, data)

# 统计与报告生成
times_arr = np.array(execution_times)

mean_time = np.mean(times_arr)
max_time = np.max(times_arr)
min_time = np.min(times_arr)
std_time = np.std(times_arr)
p99_time = np.percentile(times_arr, 99) # 99% 分位数 (99%的计算都在这个时间以内)

# 计算算力等效频率 (Hz)
equivalent_hz = 1000.0 / mean_time if mean_time > 0 else 0

print("\n==============================================")
print("             MPC 控制器性能报告               ")
print("==============================================")
print(f"样本总数      : {TEST_STEPS} steps")
print(f"平均耗时      : {mean_time:.3f} ms")
print(f"最小耗时      : {min_time:.3f} ms")
print(f"最大耗时      : {max_time:.3f} ms")
print(f"耗时标准差    : {std_time:.3f} ms")
print(f"99% 耗时阈值  : {p99_time:.3f} ms (反映卡顿极值)")
print(f"等效运行频率  : {equivalent_hz:.1f} Hz")
print("==============================================")

# 性能评估建议
if max_time > dt * 1000:
    print(f"[警告] 最大 MPC 耗时 ({max_time:.2f} ms) 超过了物理步长 ({dt*1000:.2f} ms)！在实时控制中必然会发生丢帧或延迟。")
elif mean_time > dt * 1000 * 0.8:
    print(f"[注意] 平均耗时逼近物理步长，CPU 负载非常高，建议进一步优化 C++ 代码（如降低预测步长 Horizon 或减少决策变量）。")
else:
    print(f"MPC 计算速度完全可以满足当前 dt = {dt}s 的实时仿真需求。")