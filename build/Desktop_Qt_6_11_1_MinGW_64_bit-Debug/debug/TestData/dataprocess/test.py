import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 1) 读取数据
csv_path = "BAT_Heat_Log_Data_2026_05_27_22_00_56_imu_result.csv"  # 改成你的文件名
df = pd.read_csv(csv_path)

# 2) 组装旋转矩阵 R(t) (N,3,3)
R_all = np.empty((len(df), 3, 3), dtype=float)
R_all[:, 0, 0] = df["R11"]; R_all[:, 0, 1] = df["R12"]; R_all[:, 0, 2] = df["R13"]
R_all[:, 1, 0] = df["R21"]; R_all[:, 1, 1] = df["R22"]; R_all[:, 1, 2] = df["R23"]
R_all[:, 2, 0] = df["R31"]; R_all[:, 2, 1] = df["R32"]; R_all[:, 2, 2] = df["R33"]

t = df["Time_s"].to_numpy() if "Time_s" in df.columns else np.arange(len(df))

# 3) （强烈建议）降采样，不然帧太多会很慢
step = max(1, len(df)//800)  # 目标大概 800 帧以内
R_all = R_all[::step]
t = t[::step]

# 4) 选择一根“机体指向杆”
v0 = np.array([100.0, 0.0, 0.0])   # 你想要的“默认起始100”

# 5) 预先算出杆尖端轨迹（尾迹）
tips = (R_all @ v0.reshape(3,1)).squeeze(-1)  # (N,3)

# 6) 建图
fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(111, projection="3d")

L = 110  # 坐标范围，略大于向量长度
def setup_ax():
    ax.set_xlim(-L, L); ax.set_ylim(-L, L); ax.set_zlim(-L, L)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")

def update(i):
    ax.cla()
    setup_ax()

    R = R_all[i]
    # 机体三轴（单位长度即可）
    x_axis = R[:,0]
    y_axis = R[:,1]
    z_axis = R[:,2]

    # 画机体坐标轴
    ax.quiver(0,0,0, *x_axis, color="r", linewidth=3, length=40)
    ax.quiver(0,0,0, *y_axis, color="g", linewidth=3, length=40)
    ax.quiver(0,0,0, *z_axis, color="b", linewidth=3, length=40)

    # 画“指向杆”（从原点到杆尖端）
    tip = tips[i]
    ax.plot([0, tip[0]], [0, tip[1]], [0, tip[2]], color="k", linewidth=3)

    # 画尾迹（到当前帧为止）
    ax.plot(tips[:i+1,0], tips[:i+1,1], tips[:i+1,2], color="orange", linewidth=2, alpha=0.8)

    ax.set_title(f"t = {t[i]:.2f}s  (frame {i}/{len(t)-1})")
    


print("rows:", len(df))
print("Time head/tail:", df["Time_s"].iloc[0], df["Time_s"].iloc[-1])




ani = FuncAnimation(fig, update, frames=len(t), interval=20)
plt.show()