import socket
import time
import numpy as np
import matplotlib.pyplot as plt

from gesture_optimized import (
    quaternion_to_rotation_matrix,
    RealtimeBoxViewer
)



# ============================================================
# UDP 设置
# ============================================================

UDP_IP = "127.0.0.1"
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# 减小接收缓冲区，降低旧帧积压导致的延迟
sock.setsockopt(
    socket.SOL_SOCKET,
    socket.SO_RCVBUF,
    4096
)

# 非阻塞模式：用于读空缓冲区，只保留最新帧
sock.setblocking(False)

print("UDP Ready")

# ============================================================
# 显示设置
# ============================================================

INITIAL_VIEW_DECIDED = False

plt.ion()

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

viewer = RealtimeBoxViewer(ax)

latest_values = None
last_draw_time = 0.0

# 绘图帧率：30FPS。若CPU压力大，可改为 1.0 / 20.0 或 1.0 / 15.0
DRAW_INTERVAL = 1.0 / 30.0

# ============================================================
# 初始姿态归零设置
# ============================================================

# True：启动第一帧作为显示零姿态
# False：直接显示Qt解算的绝对姿态
USE_RELATIVE_ZERO = False
R_initial = None

# ============================================================
# 初始化视角/坐标轴翻转设置
# ============================================================

# 是否根据初始化时的 acc_z 判断是否启用额外坐标翻转
USE_INIT_ACCEL_Z_FLIP = True

# 判断阈值：
# 若初始化时 acc_z < INIT_ACCEL_Z_THRESHOLD，则进入“反向初始姿态”处理
INIT_ACCEL_Z_THRESHOLD = 0.0

# 无论 acc_z 正负都始终应用的基础翻转
# 如果你想全局翻某个显示轴，可以改这里
BASE_VIEW_FLIP_X = False
BASE_VIEW_FLIP_Y = False
BASE_VIEW_FLIP_Z = False

# 当初始化 acc_z < 0 时额外应用的翻转
# 你现在不确定到底该翻哪个轴，就在这里试
#
# 例1：只翻 X
# NEG_Z_VIEW_FLIP_X = True
# NEG_Z_VIEW_FLIP_Y = False
# NEG_Z_VIEW_FLIP_Z = False
#
# 例2：翻 X 和 Y
# NEG_Z_VIEW_FLIP_X = True
# NEG_Z_VIEW_FLIP_Y = True
# NEG_Z_VIEW_FLIP_Z = False
#
# 例3：翻 X、Y、Z
# NEG_Z_VIEW_FLIP_X = True
# NEG_Z_VIEW_FLIP_Y = True
# NEG_Z_VIEW_FLIP_Z = True
NEG_Z_VIEW_FLIP_X = False
NEG_Z_VIEW_FLIP_Y = False
NEG_Z_VIEW_FLIP_Z = False

# 自动生成的显示翻转矩阵，初始化时确定一次
VIEW_FLIP_MATRIX = None
VIEW_FLIP_DECIDED = False

# ============================================================
# 提前预测补偿设置
# ============================================================

# 是否启用提前预测补偿
ENABLE_PREDICTION = True

# 提前预测时间，单位秒。
# 建议调参顺序：0.06, 0.08, 0.10, 0.12, 0.15, 0.18, 0.20
PREDICT_TIME = 0.08

# 初始化阶段不做预测，先用原始姿态建立零姿态。
# 如果启动时模型抖动，可加大到 0.8 或 1.0；如果想更快进入预测，可减小到 0.2。
INIT_NO_PREDICT_SEC = 0.2

# 如果预测方向整体反了，改成 True
INVERT_GYRO_FOR_PREDICTION = False

# 限制最大预测角，防止快速甩动时过冲。单位：度
MAX_PREDICT_ANGLE_DEG = 8

first_time_s = None
frame_count = 0

# 丢弃第1个有效数据包，从第2帧开始作为初始姿态
valid_udp_frame_count = 0
DROP_FIRST_VALID_FRAME = True

# ============================================================
# 四元数工具函数
# ============================================================

def normalize_quat(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)

    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    return q / n


def predict_quaternion_by_gyro(q, gx, gy, gz, lead_time):
    """
    使用当前四元数和当前角速度预测 lead_time 秒后的四元数。

    q: [qw, qx, qy, qz]
    gx, gy, gz: rad/s
    lead_time: seconds
    """

    q = normalize_quat(q)

    if INVERT_GYRO_FOR_PREDICTION:
        gx = -gx
        gy = -gy
        gz = -gz

    omega_norm = np.sqrt(gx * gx + gy * gy + gz * gz)

    # 限制最大预测角，防止快速运动时预测过冲
    max_angle = np.deg2rad(MAX_PREDICT_ANGLE_DEG)

    if omega_norm > 1e-6:
        predict_angle = omega_norm * lead_time

        if predict_angle > max_angle:
            lead_time = max_angle / omega_norm

    q0, q1, q2, q3 = q
    dt = lead_time

    # 与 Qt/C 端 gyro_solu() 一致的欧拉积分形式
    tq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * dt / 2.0
    tq1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * dt / 2.0
    tq2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * dt / 2.0
    tq3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * dt / 2.0

    return normalize_quat([tq0, tq1, tq2, tq3])


def build_view_flip_matrix(acc_z):
    """
    根据初始化时的 acc_z 和用户配置，构造显示翻转矩阵。

    这个矩阵只影响 Python 显示，不影响 Qt 解算。
    """

    flip_x = BASE_VIEW_FLIP_X
    flip_y = BASE_VIEW_FLIP_Y
    flip_z = BASE_VIEW_FLIP_Z

    if USE_INIT_ACCEL_Z_FLIP and acc_z < INIT_ACCEL_Z_THRESHOLD:
        # 当 acc_z < 阈值时，叠加用户配置的翻转
        flip_x = flip_x ^ NEG_Z_VIEW_FLIP_X
        flip_y = flip_y ^ NEG_Z_VIEW_FLIP_Y
        flip_z = flip_z ^ NEG_Z_VIEW_FLIP_Z

        print("Init Accel_Z < threshold, applying conditional view flip")
    else:
        print("Init Accel_Z >= threshold, using normal/base view flip")

    sx = -1.0 if flip_x else 1.0
    sy = -1.0 if flip_y else 1.0
    sz = -1.0 if flip_z else 1.0

    print(f"View flip sign: X={sx}, Y={sy}, Z={sz}")

    return np.diag([sx, sy, sz])


# ============================================================
# 主循环
# ============================================================

while True:
    # --------------------------------------------------------
    # 1. 读空 UDP 缓冲区，只保留最新一帧
    # --------------------------------------------------------
    while True:
        try:
            data, _ = sock.recvfrom(2048)
            text = data.decode().strip()
            values = list(map(float, text.split(",")))

            # Qt发送格式：time, ax, ay, az, gx, gy, gz, qw, qx, qy, qz
            if len(values) == 14:
                valid_udp_frame_count += 1

                # 自动丢弃第1个有效帧
                if DROP_FIRST_VALID_FRAME and valid_udp_frame_count == 1:
                    continue

                latest_values = values

        except BlockingIOError:
            break

        except ValueError:
            continue

    # --------------------------------------------------------
    # 2. 如果还没收到数据，等待
    # --------------------------------------------------------
    if latest_values is None:
        time.sleep(0.001)
        continue

    # --------------------------------------------------------
    # 3. 控制绘图频率
    # --------------------------------------------------------
    now = time.time()

    if now - last_draw_time < DRAW_INTERVAL:
        time.sleep(0.001)
        continue

    last_draw_time = now

    # --------------------------------------------------------
    # 4. 解析最新一帧
    # --------------------------------------------------------
    time_s = latest_values[0]

    acc_x = latest_values[1]
    acc_y = latest_values[2]
    acc_z = latest_values[3]

    gyro_x = latest_values[4]
    gyro_y = latest_values[5]
    gyro_z = latest_values[6]

    qw = latest_values[7]
    qx = latest_values[8]
    qy = latest_values[9]
    qz = latest_values[10]

    # 只在初始化时根据 acc_z 决定一次观察视角
    if not INITIAL_VIEW_DECIDED:
        viewer.set_initial_view_by_accel_z(acc_z)
        INITIAL_VIEW_DECIDED = True

    frame_count += 1

    if first_time_s is None:
        first_time_s = time_s

    elapsed_from_start = time_s - first_time_s

    # --------------------------------------------------------
    # 4.1 初始化时，根据 acc_z 确定显示翻转矩阵
    # --------------------------------------------------------
    if not VIEW_FLIP_DECIDED:
        VIEW_FLIP_MATRIX = build_view_flip_matrix(acc_z)
        VIEW_FLIP_DECIDED = True

    q_now = normalize_quat(
        np.array([qw, qx, qy, qz], dtype=float)
    )

    # --------------------------------------------------------
    # 5. 初始化阶段不用预测；初始化后可预测
    # --------------------------------------------------------
    if elapsed_from_start < INIT_NO_PREDICT_SEC:
        q_used = q_now
    else:
        if ENABLE_PREDICTION:
            q_used = predict_quaternion_by_gyro(
                q_now,
                gyro_x,
                gyro_y,
                gyro_z,
                PREDICT_TIME
            )
        else:
            q_used = q_now

    # --------------------------------------------------------
    # 6. 四元数转旋转矩阵
    # --------------------------------------------------------
    R_current = quaternion_to_rotation_matrix(
        q_used[0],
        q_used[1],
        q_used[2],
        q_used[3]
    )

    # --------------------------------------------------------
    # 7. 初始姿态归零
    # --------------------------------------------------------
    if USE_RELATIVE_ZERO:
        if R_initial is None:
            # 使用未预测的初始姿态作为零点，避免预测误差污染零点
            R_initial = quaternion_to_rotation_matrix(
                q_now[0],
                q_now[1],
                q_now[2],
                q_now[3]
            ).copy()

        R_display = R_current @ R_initial.T

        # 如果转动方向不符合直觉，可尝试下面这一行替换上一行：
        # R_display = R_initial.T @ R_current

    else:
        R_display = R_current

    # --------------------------------------------------------
    # 8. 根据初始化 acc_z 和用户配置应用显示翻转
    # --------------------------------------------------------
    if VIEW_FLIP_MATRIX is not None:
        R_display = VIEW_FLIP_MATRIX @ R_display

    # --------------------------------------------------------
    # 9. 更新模型
    # --------------------------------------------------------
    viewer.update(R_display, time_s=time_s)

    fig.canvas.draw_idle()
    plt.pause(0.001)