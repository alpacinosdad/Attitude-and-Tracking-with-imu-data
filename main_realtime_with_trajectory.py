import socket
import time
import numpy as np
import matplotlib.pyplot as plt

from gesture_optimized import quaternion_to_rotation_matrix
from trajectory_realtime import RealtimeTrajectoryEstimator
from trajectory_dashboard import RealtimeTrajectoryDashboard

# ============================================================
# UDP 设置
# ============================================================

UDP_IP = "127.0.0.1"
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
sock.setblocking(False)

print("UDP Ready: realtime trajectory dashboard")

# ============================================================
# 姿态显示参数
# ============================================================

# True：姿态显示使用第一帧作为显示零姿态
# False：直接显示Qt解算的绝对姿态
USE_RELATIVE_ZERO = False
R_initial = None

# 预测补偿只用于右上角姿态显示，不用于轨迹积分
ENABLE_PREDICTION = True
PREDICT_TIME = 0.08
INIT_NO_PREDICT_SEC = 0.2
INVERT_GYRO_FOR_PREDICTION = False
MAX_PREDICT_ANGLE_DEG = 8.0

# 丢弃第1个有效数据包，从第2帧开始作为初始姿态
DROP_FIRST_VALID_FRAME = True
valid_udp_frame_count = 0

# 绘图帧率
DRAW_INTERVAL = 1.0 / 20.0
last_draw_time = 0.0

first_time_s = None
latest_values = None
INITIAL_VIEW_DECIDED = False


def normalize_quat(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n


def predict_quaternion_by_gyro(q, gx, gy, gz, lead_time):
    q = normalize_quat(q)

    if INVERT_GYRO_FOR_PREDICTION:
        gx = -gx
        gy = -gy
        gz = -gz

    omega_norm = np.sqrt(gx * gx + gy * gy + gz * gz)
    max_angle = np.deg2rad(MAX_PREDICT_ANGLE_DEG)

    if omega_norm > 1e-6:
        predict_angle = omega_norm * lead_time
        if predict_angle > max_angle:
            lead_time = max_angle / omega_norm

    q0, q1, q2, q3 = q
    dt = lead_time

    tq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * dt / 2.0
    tq1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * dt / 2.0
    tq2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * dt / 2.0
    tq3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * dt / 2.0

    return normalize_quat([tq0, tq1, tq2, tq3])


plt.ion()
dashboard = RealtimeTrajectoryDashboard()
trajectory = RealtimeTrajectoryEstimator()

while True:
    # --------------------------------------------------------
    # 1. 读空 UDP 缓冲区，只保留最新一帧
    # --------------------------------------------------------
    while True:
        try:
            data, _ = sock.recvfrom(2048)
            text = data.decode().strip()
            values = list(map(float, text.split(",")))
            if len(values) == 14:
                valid_udp_frame_count += 1
                if DROP_FIRST_VALID_FRAME and valid_udp_frame_count == 1:
                    continue
                latest_values = values
        except BlockingIOError:
            break
        except ValueError:
            continue

    if latest_values is None:
        time.sleep(0.001)
        continue

    now = time.time()
    if now - last_draw_time < DRAW_INTERVAL:
        time.sleep(0.001)
        continue
    last_draw_time = now

    # --------------------------------------------------------
    # 2. 解析数据
    # Qt格式：time, ax, ay, az, gx, gy, gz, qw, qx, qy, qz
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

    aax = latest_values[11]
    aay = latest_values[12]
    aaz = latest_values[13]



    if first_time_s is None:
        first_time_s = time_s
    elapsed_from_start = time_s - first_time_s

    acc_body = np.array([acc_x, acc_y, acc_z], dtype=float)
    gyro_body = np.array([gyro_x, gyro_y, gyro_z], dtype=float)
    aaccel_world = np.array([aax, aay, aaz], dtype=float)
    q_now = normalize_quat(np.array([qw, qx, qy, qz], dtype=float))

    # --------------------------------------------------------
    # 3. 物理姿态：用于轨迹积分，绝不使用预测/显示视角修正
    # --------------------------------------------------------
    R_physical = quaternion_to_rotation_matrix(q_now[0], q_now[1], q_now[2], q_now[3])

    # --------------------------------------------------------
    # 4. 轨迹估计
    # --------------------------------------------------------
    pos, vel, acc_used, is_stationary = trajectory.update(
        time_s,
        acc_body,
        gyro_body,
        R_physical,
        aaccel_world=aaccel_world
    )

    # --------------------------------------------------------
    # 5. 姿态显示：可使用预测补偿，只影响右上角圆柱体
    # --------------------------------------------------------
    if not INITIAL_VIEW_DECIDED:
        dashboard.set_initial_view_by_accel_z(acc_z)
        INITIAL_VIEW_DECIDED = True

    if elapsed_from_start < INIT_NO_PREDICT_SEC:
        q_display = q_now
    else:
        if ENABLE_PREDICTION:
            q_display = predict_quaternion_by_gyro(
                q_now,
                gyro_x,
                gyro_y,
                gyro_z,
                PREDICT_TIME
            )
        else:
            q_display = q_now

    R_current_display = quaternion_to_rotation_matrix(
        q_display[0],
        q_display[1],
        q_display[2],
        q_display[3]
    )

    if USE_RELATIVE_ZERO:
        if R_initial is None:
            R_initial = R_physical.copy()
        R_display = R_current_display @ R_initial.T
    else:
        R_display = R_current_display

    # --------------------------------------------------------
    # 6. 更新大图显示
    # --------------------------------------------------------
    histories = trajectory.get_histories()
    dashboard.update(time_s, R_display, histories, is_stationary)
