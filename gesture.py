import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import sys

# 修复 Windows 中文打印报错
try:
    sys.stdout.reconfigure(encoding='utf-8')
except AttributeError:
    pass

# ==========================================
# 1. 加载你自己的真实数据
# ==========================================
# 请确保这里的路径是你真实的表格路径
file_path = "BAT_Heat_Log_Data_2026_06_08_13_39_53_imu_result.csv"  # 改成你的文件名
df = pd.read_csv(file_path)

num_frames = len(df)

# 如果你的表格使用的是列向量（请根据实际情况保留或修改）
USE_ROW_AS_AXIS = True 

# ==========================================
# 2. 搭建 3D 观察环境
# ==========================================
fig = plt.figure(figsize=(10, 8))
# 留出底部空间给滑动条
plt.subplots_adjust(bottom=0.25) 
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim([-1.2, 1.2]); ax.set_ylim([-1.2, 1.2]); ax.set_zlim([-1.2, 1.2])
ax.set_xlabel('Global X'); ax.set_ylabel('Global Y'); ax.set_zlabel('Global Z')
ax.set_title('Manual Hardware Attitude Inspector (Read from YOUR Data)')

# 初始化你的硬件三轴
x_axis, = ax.plot([], [], [], color='red', linewidth=5, label='Hardware X (Forward)')
y_axis, = ax.plot([], [], [], color='green', linewidth=2, label='Hardware Y')
z_axis, = ax.plot([], [], [], color='blue', linewidth=2, label='Hardware Z')

# 文本显示当前读取的数据行详情
info_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=10, 
                      bbox=dict(facecolor='white', alpha=0.9))
ax.legend(loc='upper right')

# ==========================================
# 3. 核心功能：根据你选择的行数，更新姿态
# ==========================================
def update_plot(frame_idx):
    frame_idx = int(frame_idx)
    row = df.iloc[frame_idx]
    
    # 从你的表格中提取真正的 R 矩阵
    if USE_ROW_AS_AXIS:
        rot_x = np.array([row['R11'], row['R12'], row['R13']])
        rot_y = np.array([row['R21'], row['R22'], row['R23']])
        rot_z = np.array([row['R31'], row['R32'], row['R33']])
    else:
        rot_x = np.array([row['R11'], row['R21'], row['R31']])
        rot_y = np.array([row['R12'], row['R22'], row['R32']])
        rot_z = np.array([row['R13'], row['R23'], row['R33']])
        
    # 画线
    x_axis.set_data_3d([0, rot_x[0]], [0, rot_x[1]], [0, rot_x[2]])
    y_axis.set_data_3d([0, rot_y[0]], [0, rot_y[1]], [0, rot_y[2]])
    z_axis.set_data_3d([0, rot_z[0]], [0, rot_z[1]], [0, rot_z[2]])
    
    # 实时显示你表格里的具体数值
    info_text.set_text(
        f"Data Row: {frame_idx}\n"
        f"Time: {row['Time_s']} s\n"
        f"Euler (Roll/Pitch/Yaw): {row.get('roll', 0):.1f}, {row.get('pitch', 0):.1f}, {row.get('yaw', 0):.1f}"
    )
    fig.canvas.draw_idle()

# ==========================================
# 4. 添加手动滑动条 (让你自己控制进度)
# ==========================================
ax_slider = plt.axes([0.15, 0.1, 0.65, 0.03])
frame_slider = Slider(
    ax=ax_slider,
    label='Data Row',
    valmin=0,
    valmax=num_frames - 1,
    valinit=0,
    valstep=1 # 每次严格滑动一行
)

# 绑定滑动事件
frame_slider.on_changed(update_plot)

# 初始化第一帧
update_plot(0)

plt.show()