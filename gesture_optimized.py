import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ============================================================
# 基础参数
# ============================================================

CSV_PATH = "BAT_Heat_Log_Data_2026_06_26_18_17_00_imu_result.csv"
ROW_INDEX = 0
USE_NEGATIVE_ACCEL = False

# 原长方体 X 轴长轴；实时圆柱体也沿 X 轴
BOX_LENGTH_X = 20.0
BOX_WIDTH_Y = 5.0
BOX_HEIGHT_Z = 2.0

# 传感器坐标系到显示世界坐标系的映射
SENSOR_TO_WORLD = np.array([
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0]
], dtype=float)


# ============================================================
# 离线 CSV 兼容函数
# ============================================================

def load_attitude_df(csv_path):
    df = pd.read_csv(csv_path, encoding="utf-8-sig")
    df.columns = [str(c).strip().lower() for c in df.columns]
    return df


def quaternion_to_rotation_matrix(qw, qx, qy, qz):
    q = np.array([qw, qx, qy, qz], dtype=float)
    norm = np.linalg.norm(q)

    if norm < 1e-12:
        return np.eye(3)

    q = q / norm
    w, x, y, z = q

    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ], dtype=float)


def get_rotation_matrix_from_row(row):
    matrix_cols = [
        "r11", "r12", "r13",
        "r21", "r22", "r23",
        "r31", "r32", "r33"
    ]

    if all(col in row.index for col in matrix_cols):
        return np.array([
            [row["r11"], row["r12"], row["r13"]],
            [row["r21"], row["r22"], row["r23"]],
            [row["r31"], row["r32"], row["r33"]],
        ], dtype=float)

    quat_cols = ["qw", "qx", "qy", "qz"]
    if all(col in row.index for col in quat_cols):
        return quaternion_to_rotation_matrix(
            row["qw"], row["qx"], row["qy"], row["qz"]
        )

    alt_quat_cols = ["q0", "q1", "q2", "q3"]
    if all(col in row.index for col in alt_quat_cols):
        return quaternion_to_rotation_matrix(
            row["q0"], row["q1"], row["q2"], row["q3"]
        )

    raise ValueError("CSV must contain R11..R33 or qw,qx,qy,qz or q0..q3.")


# ============================================================
# 坐标变换工具
# ============================================================

def sensor_to_world(points_sensor):
    points_sensor = np.asarray(points_sensor, dtype=float)
    shape = points_sensor.shape
    flat = points_sensor.reshape(-1, 3)
    world = flat @ SENSOR_TO_WORLD.T
    return world.reshape(shape)


def transform_sensor_object(points_sensor, R_sensor):
    points_sensor = np.asarray(points_sensor, dtype=float)
    flat = points_sensor.reshape(-1, 3)
    rotated = flat @ R_sensor.T
    world = sensor_to_world(rotated)
    return world.reshape(points_sensor.shape)


def set_axes_equal(ax, limit=22.0):
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)

    try:
        ax.set_box_aspect([1, 1, 1])
    except Exception:
        pass


# ============================================================
# 原始长方体离线绘图函数：保留，方便 CSV/Slider 继续使用
# ============================================================

def create_box_along_sensor_axes(length_x=BOX_LENGTH_X,
                                 width_y=BOX_WIDTH_Y,
                                 height_z=BOX_HEIGHT_Z):
    x0, x1 = -length_x / 2.0, length_x / 2.0
    y0, y1 = -width_y / 2.0, width_y / 2.0
    z0, z1 = -height_z / 2.0, height_z / 2.0

    faces = []

    faces.append(np.stack([
        np.array([[x0, x1], [x0, x1]]),
        np.array([[y0, y0], [y1, y1]]),
        np.array([[z0, z0], [z0, z0]])
    ], axis=-1))

    faces.append(np.stack([
        np.array([[x0, x1], [x0, x1]]),
        np.array([[y0, y0], [y1, y1]]),
        np.array([[z1, z1], [z1, z1]])
    ], axis=-1))

    faces.append(np.stack([
        np.array([[x0, x0], [x0, x0]]),
        np.array([[y0, y1], [y0, y1]]),
        np.array([[z0, z0], [z1, z1]])
    ], axis=-1))

    faces.append(np.stack([
        np.array([[x1, x1], [x1, x1]]),
        np.array([[y0, y1], [y0, y1]]),
        np.array([[z0, z0], [z1, z1]])
    ], axis=-1))

    faces.append(np.stack([
        np.array([[x0, x1], [x0, x1]]),
        np.array([[y0, y0], [y0, y0]]),
        np.array([[z0, z1], [z0, z1]])
    ], axis=-1))

    faces.append(np.stack([
        np.array([[x0, x1], [x0, x1]]),
        np.array([[y1, y1], [y1, y1]]),
        np.array([[z0, z1], [z0, z1]])
    ], axis=-1))

    return faces


def plot_attitude_on_axes(ax, R_sensor, row_index=None, row=None):
    ax.cla()

    faces_sensor = create_box_along_sensor_axes()
    face_colors = ["#4C9FD6"] * len(faces_sensor)
    face_colors[3] = "red"

    for face, color in zip(faces_sensor, face_colors):
        face_world = transform_sensor_object(face, R_sensor)
        X = face_world[:, :, 0]
        Y = face_world[:, :, 1]
        Z = face_world[:, :, 2]
        ax.plot_surface(
            X, Y, Z,
            color=color,
            alpha=1.0,
            linewidth=0,
            antialiased=True,
            shade=True
        )

    axes_sensor = np.array([
        [0.0, 0.0, 0.0],
        [5.0, 0.0, 0.0],
        [0.0, 5.0, 0.0],
        [0.0, 0.0, 5.0]
    ], dtype=float)

    axes_world = transform_sensor_object(axes_sensor, R_sensor)
    origin = axes_world[0]

    ax.plot(
        [origin[0], axes_world[1, 0]],
        [origin[1], axes_world[1, 1]],
        [origin[2], axes_world[1, 2]],
        color="red",
        linewidth=3
    )

    ax.plot(
        [origin[0], axes_world[2, 0]],
        [origin[1], axes_world[2, 1]],
        [origin[2], axes_world[2, 2]],
        color="green",
        linewidth=3
    )

    ax.plot(
        [origin[0], axes_world[3, 0]],
        [origin[1], axes_world[3, 1]],
        [origin[2], axes_world[3, 2]],
        color="blue",
        linewidth=3
    )

    ax.set_axis_off()
    ax.view_init(elev=22, azim=35)
    set_axes_equal(ax, limit=22.0)

    title = f"Box Attitude - Row {row_index}"
    if row is not None and "time_s" in row.index:
        title += f"  Time {row['time_s']:.3f}s"
    ax.set_title(title)


# ============================================================
# 实时圆柱体模型：长轴沿传感器 X 轴
# ============================================================

def create_cylinder_mesh(length_x=BOX_LENGTH_X,
                         radius=BOX_WIDTH_Y / 2.0,
                         segments=48):
    x0, x1 = -length_x / 2.0, length_x / 2.0

    theta = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    y = radius * np.cos(theta)
    z = radius * np.sin(theta)

    left_ring = np.stack([
        np.full(segments, x0),
        y,
        z
    ], axis=1)

    right_ring = np.stack([
        np.full(segments, x1),
        y,
        z
    ], axis=1)

    vertices = np.vstack([
        left_ring,
        right_ring
    ])

    faces = []

    for i in range(segments):
        j = (i + 1) % segments
        faces.append([
            i,
            j,
            segments + j,
            segments + i
        ])

    # 左端盖和右端盖
    faces.append(list(range(segments - 1, -1, -1)))
    faces.append(list(range(segments, 2 * segments)))

    return vertices, faces


class RealtimeBoxViewer:
    """实时显示类：圆柱体沿 X 轴，不做预测补偿。"""

    def __init__(self, ax):
        self.ax = ax

        self.vertices_sensor, self.face_indices = create_cylinder_mesh(
            length_x=BOX_LENGTH_X,
            radius=BOX_WIDTH_Y / 2.0,
            segments=48
        )

        self.face_colors = ["#4C9FD6"] * len(self.face_indices)

        # 右端盖标红，代表原 X 正方向。如果反了，改成 self.face_colors[-2] = "red"
        self.face_colors[-1] = "red"

        R0 = np.eye(3)
        verts_world = self._get_world_vertices(R0)
        poly_faces = [verts_world[idxs] for idxs in self.face_indices]

        self.body = Poly3DCollection(
            poly_faces,
            facecolors=self.face_colors,
            edgecolors="none",
            linewidths=0,
            alpha=1.0
        )

        self.ax.add_collection3d(self.body)

        # 只保留红色 X 轴方向线；不画绿色和蓝色轴
        self.x_line, = self.ax.plot([], [], [], color="red", linewidth=3)

        self.ax.set_axis_off()
        self.front_elev = 22
        self.front_azim = 35

        self.back_elev = 22
        self.back_azim = 215

        self.current_view_mode = "front"
        self.ax.view_init(elev=self.front_elev, azim=self.front_azim)

        set_axes_equal(self.ax, limit=22.0)
        self.ax.set_title("Realtime Attitude")

        self.title_count = 0

    def _get_world_vertices(self, R_sensor):
        rotated = self.vertices_sensor @ R_sensor.T
        world = sensor_to_world(rotated)
        return world
    
    def set_initial_view_by_accel_z(self, acc_z):
        """
        只在初始化时调用一次。

        如果初始 acc_z < 0：
            后续固定为背面视角。
        否则：
            后续固定为正面视角。
        """

        if acc_z < 0:
            self.view_mode = "back"
            self.ax.view_init(
                elev=self.back_elev,
                azim=self.back_azim
            )
            print("Initial view: BACK, acc_z =", acc_z)
        else:
            self.view_mode = "front"
            self.ax.view_init(
                elev=self.front_elev,
                azim=self.front_azim
            )
            print("Initial view: FRONT, acc_z =", acc_z)


    
    def update(self, R_sensor, time_s=None):
        verts_world = self._get_world_vertices(R_sensor)
        poly_faces = [verts_world[idxs] for idxs in self.face_indices]
        self.body.set_verts(poly_faces)

        axes_sensor = np.array([
            [0.0, 0.0, 0.0],
            [6.0, 0.0, 0.0]
        ], dtype=float)

        rotated_axes = axes_sensor @ R_sensor.T
        axes_world = sensor_to_world(rotated_axes)
        origin = axes_world[0]

        self.x_line.set_data(
            [origin[0], axes_world[1, 0]],
            [origin[1], axes_world[1, 1]]
        )
        self.x_line.set_3d_properties(
            [origin[2], axes_world[1, 2]]
        )

        self.title_count += 1
        if time_s is not None and self.title_count % 20 == 0:
            self.ax.set_title(f"Realtime Attitude  t={time_s:.2f}s")
