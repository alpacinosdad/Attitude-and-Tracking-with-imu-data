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

CSV_PATH = "shuzhi_low_nomove.csv"
ROW_INDEX = 0
USE_NEGATIVE_ACCEL = False

BOX_LENGTH_X = 20.0
BOX_WIDTH_Y = 5.0
BOX_HEIGHT_Z = 2.0


# ...existing code...

def load_acc_df(csv_path):
    df = pd.read_csv(csv_path, encoding="utf-8-sig")
    df.columns = [str(c).strip().lower() for c in df.columns]

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

    return df, selected_cols


def get_acc_from_df(df, selected_cols, row_index):
    if row_index < 0 or row_index >= len(df):
        raise IndexError(
            f"row_index out of range. CSV row count = {len(df)}, "
            f"selected row_index = {row_index}"
        )
    row = df.iloc[row_index]
    return np.array([row[selected_cols[0]], row[selected_cols[1]], row[selected_cols[2]]], dtype=float)


def normalize(v):
    v = np.asarray(v, dtype=float)
    norm = np.linalg.norm(v)
    if norm < 1e-12:
        raise ValueError("Zero-length vector cannot be normalized")
    return v / norm

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
        tmp = np.array([1.0, 0.0, 0.0], dtype=float)
        if abs(np.dot(a, tmp)) > 0.9:
            tmp = np.array([0.0, 1.0, 0.0], dtype=float)
        axis = normalize(np.cross(a, tmp))
        K = skew(axis)
        return np.eye(3) + 2.0 * (K @ K)
    v = np.cross(a, b)
    s = np.linalg.norm(v)
    K = skew(v)
    return np.eye(3) + K + K @ K * ((1.0 - dot) / (s**2))

def sensor_to_world(points_sensor):
    points_sensor = np.asarray(points_sensor, dtype=float)
    shape = points_sensor.shape
    flat = points_sensor.reshape(-1, 3)
    world = flat @ np.array([
        [0.0, 0.0, -1.0],
        [0.0, 1.0,  0.0],
        [1.0, 0.0,  0.0]
    ], dtype=float).T
    return world.reshape(shape)

def create_box_along_sensor_axes(length_x=BOX_LENGTH_X, width_y=BOX_WIDTH_Y, height_z=BOX_HEIGHT_Z):
    x0, x1 = -length_x/2.0, length_x/2.0
    y0, y1 = -width_y/2.0, width_y/2.0
    z0, z1 = -height_z/2.0, height_z/2.0
    faces = []
    faces.append(np.stack([np.array([[x0, x1],[x0, x1]]),
                           np.array([[y0, y0],[y1, y1]]),
                           np.array([[z0, z0],[z0, z0]])], axis=-1))
    faces.append(np.stack([np.array([[x0, x1],[x0, x1]]),
                           np.array([[y0, y0],[y1, y1]]),
                           np.array([[z1, z1],[z1, z1]])], axis=-1))
    faces.append(np.stack([np.array([[x0, x0],[x0, x0]]),
                           np.array([[y0, y1],[y0, y1]]),
                           np.array([[z0, z0],[z1, z1]])], axis=-1))
    faces.append(np.stack([np.array([[x1, x1],[x1, x1]]),
                           np.array([[y0, y1],[y0, y1]]),
                           np.array([[z0, z0],[z1, z1]])], axis=-1))
    faces.append(np.stack([np.array([[x0, x1],[x0, x1]]),
                           np.array([[y0, y0],[y0, y0]]),
                           np.array([[z0, z1],[z0, z1]])], axis=-1))
    faces.append(np.stack([np.array([[x0, x1],[x0, x1]]),
                           np.array([[y1, y1],[y1, y1]]),
                           np.array([[z0, z1],[z0, z1]])], axis=-1))
    return faces

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

def plot_attitude_on_axes(ax, acc_current, row_index=None):
    ax.cla()

    if USE_NEGATIVE_ACCEL:
        acc_current = -acc_current

    ref_vec_sensor = np.array([1.0, 0.0, 0.0], dtype=float)
    cur_vec_sensor = normalize(acc_current)
    R_sensor = rotation_matrix_from_vectors(ref_vec_sensor, cur_vec_sensor)

    faces_sensor = create_box_along_sensor_axes(
        length_x=BOX_LENGTH_X,
        width_y=BOX_WIDTH_Y,
        height_z=BOX_HEIGHT_Z
    )

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


# ============================================================
# 6. Main
# ============================================================



if __name__ == "__main__":
    df, selected_cols = load_acc_df(CSV_PATH)
    num_rows = len(df)

    fig = plt.figure(figsize=(10, 8))
    plt.subplots_adjust(bottom=0.22)

    ax = fig.add_subplot(111, projection="3d")
    slider_ax = fig.add_axes([0.15, 0.08, 0.7, 0.04])
    row_slider = Slider(
        ax=slider_ax,
        label="CSV Row",
        valmin=0,
        valmax=num_rows - 1,
        valinit=ROW_INDEX,
        valstep=1
    )

    def on_slider(val):
        idx = int(val)
        acc = get_acc_from_df(df, selected_cols, idx)
        plot_attitude_on_axes(ax, acc, row_index=idx)
        fig.canvas.draw_idle()

    row_slider.on_changed(on_slider)
    on_slider(ROW_INDEX)
    plt.show()