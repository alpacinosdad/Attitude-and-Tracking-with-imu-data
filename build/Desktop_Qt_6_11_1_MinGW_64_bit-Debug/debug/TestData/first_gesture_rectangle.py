import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# 0. Parameters
# ============================================================

CSV_PATH = "BAT_Heat_Log_Data_2026_06_09_15_16_06.csv"

ROW_INDEX = 800

USE_NEGATIVE_ACCEL = False

BOX_LENGTH_X = 20.0
BOX_WIDTH_Y = 5.0
BOX_HEIGHT_Z = 2.0


# ============================================================
# 1. Sensor frame to world/display frame mapping
# ============================================================

SENSOR_TO_WORLD = np.array([
    [0.0, 0.0, -1.0],
    [0.0, 1.0,  0.0],
    [1.0, 0.0,  0.0]
], dtype=float)


def sensor_to_world(points_sensor):
    points_sensor = np.asarray(points_sensor, dtype=float)
    original_shape = points_sensor.shape

    flat = points_sensor.reshape(-1, 3)
    world = flat @ SENSOR_TO_WORLD.T

    return world.reshape(original_shape)


# ============================================================
# 2. Math utilities
# ============================================================

def normalize(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)

    if n < 1e-12:
        raise ValueError("Vector norm is too small to normalize.")

    return v / n


def skew(v):
    x, y, z = v

    return np.array([
        [0.0, -z,   y],
        [z,    0.0, -x],
        [-y,   x,   0.0]
    ], dtype=float)


def rotation_matrix_from_vectors(a, b):
    a = normalize(a)
    b = normalize(b)

    dot = np.dot(a, b)

    if dot > 0.999999:
        return np.eye(3)

    if dot < -0.999999:
        tmp = np.array([0.0, 1.0, 0.0], dtype=float)
        if abs(np.dot(a, tmp)) > 0.99:
            tmp = np.array([0.0, 0.0, 1.0], dtype=float)

        axis = normalize(np.cross(a, tmp))
        K = skew(axis)
        return np.eye(3) + 2.0 * (K @ K)

    v = np.cross(a, b)
    s = np.linalg.norm(v)
    c = dot

    K = skew(v)
    R = np.eye(3) + K + K @ K * ((1.0 - c) / (s ** 2))

    return R


# ============================================================
# 3. Box geometry
# ============================================================

def create_box_along_sensor_axes(
    length_x=BOX_LENGTH_X,
    width_y=BOX_WIDTH_Y,
    height_z=BOX_HEIGHT_Z
):
    x0 = -length_x / 2.0
    x1 = +length_x / 2.0
    y0 = -width_y / 2.0
    y1 = +width_y / 2.0
    z0 = -height_z / 2.0
    z1 = +height_z / 2.0

    # 6 faces of the box, each is a 2x2 grid
    faces = []

    xx = np.array([[x0, x1], [x0, x1]])
    yy = np.array([[y0, y0], [y1, y1]])
    zz = np.array([[z0, z0], [z0, z0]])
    faces.append(np.stack([xx, yy, zz], axis=-1))

    zz = np.array([[z1, z1], [z1, z1]])
    faces.append(np.stack([xx, yy, zz], axis=-1))

    xx = np.array([[x0, x0], [x0, x0]])
    yy = np.array([[y0, y1], [y0, y1]])
    zz = np.array([[z0, z0], [z1, z1]])
    faces.append(np.stack([xx, yy, zz], axis=-1))

    xx = np.array([[x1, x1], [x1, x1]])
    faces.append(np.stack([xx, yy, zz], axis=-1))

    xx = np.array([[x0, x1], [x0, x1]])
    yy = np.array([[y0, y0], [y0, y0]])
    zz = np.array([[z0, z1], [z0, z1]])
    faces.append(np.stack([xx, yy, zz], axis=-1))

    yy = np.array([[y1, y1], [y1, y1]])
    faces.append(np.stack([xx, yy, zz], axis=-1))

    return faces


def transform_sensor_object(points_sensor, R_sensor):
    points_sensor = np.asarray(points_sensor, dtype=float)
    original_shape = points_sensor.shape

    flat = points_sensor.reshape(-1, 3)
    rotated_sensor = flat @ R_sensor.T
    world = sensor_to_world(rotated_sensor)

    return world.reshape(original_shape)


# ============================================================
# 4. CSV reading
# ============================================================

def read_acc_from_csv(csv_path, row_index):
    df = pd.read_csv(csv_path, encoding="utf-8-sig")
    df.columns = [str(c).strip().lower() for c in df.columns]

    print("CSV columns:")
    print(df.columns.tolist())

    possible_acc_cols = [
        ("accx", "accy", "accz"),
        ("accel_x", "accel_y", "accel_z"),
        ("acc_x", "acc_y", "acc_z"),
        ("ax", "ay", "az"),
    ]

    selected_cols = None
    for cols in possible_acc_cols:
        if all(col in df.columns for col in cols):
            selected_cols = cols
            break

    if selected_cols is None:
        raise ValueError(
            "Acceleration columns not found.\n"
            f"Current CSV columns are: {df.columns.tolist()}\n"
            "Please check whether the CSV contains accx/accy/accz "
            "or accel_x/accel_y/accel_z."
        )

    col_x, col_y, col_z = selected_cols

    if row_index < 0 or row_index >= len(df):
        raise IndexError(
            f"row_index out of range. CSV row count = {len(df)}, "
            f"selected row_index = {row_index}"
        )

    row = df.iloc[row_index]
    acc = np.array([row[col_x], row[col_y], row[col_z]], dtype=float)

    print(f"Using acceleration columns: {col_x}, {col_y}, {col_z}")
    print(f"CSV row {row_index} acceleration data: {acc}")

    return acc


# ============================================================
# 5. Plotting
# ============================================================

def set_axes_equal(ax, limit=22.0):
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    try:
        ax.set_box_aspect([1, 1, 1])
    except Exception:
        pass


# ...existing code...
# ...existing code...
def plot_attitude(acc_current, row_index):
    acc_current = np.asarray(acc_current, dtype=float)

    if USE_NEGATIVE_ACCEL:
        acc_current = -acc_current

    ref_vec_sensor = np.array([1.0, 0.0, 0.0], dtype=float)
    cur_vec_sensor = normalize(acc_current)
    R_sensor = rotation_matrix_from_vectors(ref_vec_sensor, cur_vec_sensor)

    print("")
    print("========== ATTITUDE DEBUG ==========")
    print("Raw acceleration acc =", acc_current)
    print("Normalized acceleration direction =", cur_vec_sensor)
    print("Reference direction in sensor frame =", ref_vec_sensor)
    print("Rotation matrix R_sensor =")
    print(R_sensor)

    faces_sensor = create_box_along_sensor_axes(
        length_x=BOX_LENGTH_X,
        width_y=BOX_WIDTH_Y,
        height_z=BOX_HEIGHT_Z
    )

    face_colors = ["#4C9FD6"] * len(faces_sensor)
    face_colors[3] = "red"  # 将一端面涂成红色

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")

    for face, color in zip(faces_sensor, face_colors):
        face_world = transform_sensor_object(face, R_sensor)
        X = face_world[:, :, 0]
        Y = face_world[:, :, 1]
        Z = face_world[:, :, 2]

        ax.plot_surface(
            X,
            Y,
            Z,
            color=color,
            alpha=1.0,
            linewidth=0,
            antialiased=True,
            shade=True
        )

    # 绘制当前姿态下的传感器坐标轴
    axes_sensor = np.array([
        [0.0, 0.0, 0.0],
        [5.0, 0.0, 0.0],
        [0.0, 5.0, 0.0],
        [0.0, 0.0, 5.0]
    ], dtype=float)
    axes_world = transform_sensor_object(axes_sensor, R_sensor)
    origin = axes_world[0]
    ax.plot([origin[0], axes_world[1, 0]], [origin[1], axes_world[1, 1]], [origin[2], axes_world[1, 2]], color="red", linewidth=3)
    ax.plot([origin[0], axes_world[2, 0]], [origin[1], axes_world[2, 1]], [origin[2], axes_world[2, 2]], color="green", linewidth=3)
    ax.plot([origin[0], axes_world[3, 0]], [origin[1], axes_world[3, 1]], [origin[2], axes_world[3, 2]], color="blue", linewidth=3)

    ax.set_axis_off()
    ax.view_init(elev=22, azim=35)
    set_axes_equal(ax, limit=22.0)

    if row_index is None:
        ax.set_title("Box Attitude")
    else:
        ax.set_title(f"Box Attitude - CSV Row {row_index}")

    plt.tight_layout()
    plt.show()

    return R_sensor
# ...existing code...


# ============================================================
# 6. Main
# ============================================================

if __name__ == "__main__":
    acc = read_acc_from_csv(CSV_PATH, ROW_INDEX)
    plot_attitude(acc, row_index=ROW_INDEX)