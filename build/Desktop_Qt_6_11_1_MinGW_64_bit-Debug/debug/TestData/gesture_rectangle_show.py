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

CSV_PATH = "BAT_Heat_Log_Data_2026_06_15_18_46_01_imu_result.csv"
ROW_INDEX = 0
USE_NEGATIVE_ACCEL = False

BOX_LENGTH_X = 20.0
BOX_WIDTH_Y = 5.0
BOX_HEIGHT_Z = 2.0

SENSOR_TO_WORLD = np.array([
    [0.0, 1.0, 0.0],
    [-1.0, 0.0,  0.0],
    [0.0, 0.0,  1.0]
], dtype=float)


def load_attitude_df(csv_path):
    df = pd.read_csv(csv_path, encoding="utf-8-sig")
    df.columns = [str(c).strip().lower() for c in df.columns]
    return df


def quaternion_to_rotation_matrix(qw, qx, qy, qz):
    q = np.array([qw, qx, qy, qz], dtype=float)
    q = q / np.linalg.norm(q)
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

    raise ValueError("CSV must contain R11..R33 or qw,qx,qy,qz (or q0..q3).")


def create_box_along_sensor_axes(length_x=BOX_LENGTH_X, width_y=BOX_WIDTH_Y, height_z=BOX_HEIGHT_Z):
    x0, x1 = -length_x / 2.0, length_x / 2.0
    y0, y1 = -width_y / 2.0, width_y / 2.0
    z0, z1 = -height_z / 2.0, height_z / 2.0
    faces = []
    faces.append(np.stack([np.array([[x0, x1], [x0, x1]]),
                           np.array([[y0, y0], [y1, y1]]),
                           np.array([[z0, z0], [z0, z0]])], axis=-1))
    faces.append(np.stack([np.array([[x0, x1], [x0, x1]]),
                           np.array([[y0, y0], [y1, y1]]),
                           np.array([[z1, z1], [z1, z1]])], axis=-1))
    faces.append(np.stack([np.array([[x0, x0], [x0, x0]]),
                           np.array([[y0, y1], [y0, y1]]),
                           np.array([[z0, z0], [z1, z1]])], axis=-1))
    faces.append(np.stack([np.array([[x1, x1], [x1, x1]]),
                           np.array([[y0, y1], [y0, y1]]),
                           np.array([[z0, z0], [z1, z1]])], axis=-1))
    faces.append(np.stack([np.array([[x0, x1], [x0, x1]]),
                           np.array([[y0, y0], [y0, y0]]),
                           np.array([[z0, z1], [z0, z1]])], axis=-1))
    faces.append(np.stack([np.array([[x0, x1], [x0, x1]]),
                           np.array([[y1, y1], [y1, y1]]),
                           np.array([[z0, z1], [z0, z1]])], axis=-1))
    return faces


def transform_sensor_object(points_sensor, R_sensor):
    points_sensor = np.asarray(points_sensor, dtype=float)
    flat = points_sensor.reshape(-1, 3)
    rotated = flat @ R_sensor.T
    world = flat @ SENSOR_TO_WORLD.T if False else sensor_to_world(rotated)
    return world.reshape(points_sensor.shape)


def sensor_to_world(points_sensor):
    points_sensor = np.asarray(points_sensor, dtype=float)
    shape = points_sensor.shape
    flat = points_sensor.reshape(-1, 3)
    world = flat @ SENSOR_TO_WORLD.T
    return world.reshape(shape)


def set_axes_equal(ax, limit=22.0):
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    try:
        ax.set_box_aspect([1, 1, 1])
    except Exception:
        pass


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
        ax.plot_surface(X, Y, Z, color=color, alpha=1.0, linewidth=0, antialiased=True, shade=True)

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

    title = f"Box Attitude - Row {row_index}"
    if row is not None and "time_s" in row.index:
        title += f"  Time {row['time_s']:.3f}s"
    ax.set_title(title)


if __name__ == "__main__":
    df = load_attitude_df(CSV_PATH)
    num_rows = len(df)

    fig = plt.figure(figsize=(10, 8))
    plt.subplots_adjust(bottom=0.22)
    ax = fig.add_subplot(111, projection="3d")

    slider_ax = fig.add_axes([0.15, 0.08, 0.7, 0.04])
    row_slider = Slider(ax=slider_ax, label="Data Row", valmin=0, valmax=num_rows - 1,
                        valinit=ROW_INDEX, valstep=1)

    def on_slider(val):
        idx = int(val)
        row = df.iloc[idx]
        R = get_rotation_matrix_from_row(row)
        plot_attitude_on_axes(ax, R, row_index=idx, row=row)
        fig.canvas.draw_idle()

    row_slider.on_changed(on_slider)
    on_slider(ROW_INDEX)
    plt.show()