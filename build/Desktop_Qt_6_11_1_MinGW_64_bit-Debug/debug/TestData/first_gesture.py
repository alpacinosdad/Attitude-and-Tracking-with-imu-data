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

# Python index starts from 0.
# ROW_INDEX = 0 means the first data row.
ROW_INDEX = 1500

# If the attitude is completely opposite, set this to True.
USE_NEGATIVE_ACCEL = False

# Cylinder parameters
CYLINDER_LENGTH = 2.4
CYLINDER_RADIUS = 0.25


# ============================================================
# 1. Sensor frame to world/display frame mapping
# ============================================================
#
# Your real initial definition:
#   accel_x around +9800 means the cylinder is vertical upward.
#
# Matplotlib uses world Z as vertical.
#
# Therefore:
#   sensor +X -> world +Z
#   sensor +Y -> world +Y
#   sensor +Z -> world -X
#
# For any point p_sensor:
#   p_world = SENSOR_TO_WORLD @ p_sensor

SENSOR_TO_WORLD = np.array([
    [0.0, 0.0, -1.0],
    [0.0, 1.0,  0.0],
    [1.0, 0.0,  0.0]
], dtype=float)


def sensor_to_world(points_sensor):
    """
    Convert points from sensor/body frame to world/display frame.
    """

    points_sensor = np.asarray(points_sensor, dtype=float)
    original_shape = points_sensor.shape

    flat = points_sensor.reshape(-1, 3)
    world = flat @ SENSOR_TO_WORLD.T

    return world.reshape(original_shape)


# ============================================================
# 2. Math utilities
# ============================================================

def normalize(v):
    """
    Normalize a vector.
    """

    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)

    if n < 1e-12:
        raise ValueError("Vector norm is too small to normalize.")

    return v / n


def skew(v):
    """
    Create skew-symmetric matrix from a 3D vector.
    """

    x, y, z = v

    return np.array([
        [0.0, -z,   y],
        [z,    0.0, -x],
        [-y,   x,   0.0]
    ], dtype=float)


def rotation_matrix_from_vectors(a, b):
    """
    Compute rotation matrix R such that:

        R @ a = b

    Both a and b are vectors in the sensor/body frame.

    This gives the shortest rotation from vector a to vector b.
    """

    a = normalize(a)
    b = normalize(b)

    dot = np.dot(a, b)

    # Almost same direction
    if dot > 0.999999:
        return np.eye(3)

    # Almost opposite direction
    if dot < -0.999999:
        tmp = np.array([0.0, 1.0, 0.0], dtype=float)

        if abs(np.dot(a, tmp)) > 0.99:
            tmp = np.array([0.0, 0.0, 1.0], dtype=float)

        axis = normalize(np.cross(a, tmp))
        K = skew(axis)

        # 180-degree rotation matrix
        return np.eye(3) + 2.0 * (K @ K)

    v = np.cross(a, b)
    s = np.linalg.norm(v)
    c = dot

    K = skew(v)

    # Rodrigues rotation formula
    R = np.eye(3) + K + K @ K * ((1.0 - c) / (s ** 2))

    return R


# ============================================================
# 3. Cylinder geometry
# ============================================================

def create_cylinder_along_sensor_x(
    length=CYLINDER_LENGTH,
    radius=CYLINDER_RADIUS,
    n_theta=120,
    n_length=100
):
    """
    Create a cylinder in sensor/body frame.

    Cylinder local coordinate:
        X: cylinder axis
        YZ: circular cross section

    Cylinder center is at origin.
    X range:
        [-length / 2, +length / 2]
    """

    x_vals = np.linspace(-length / 2.0, length / 2.0, n_length)
    theta_vals = np.linspace(0.0, 2.0 * np.pi, n_theta)

    x_grid, theta_grid = np.meshgrid(x_vals, theta_vals)

    y_grid = radius * np.cos(theta_grid)
    z_grid = radius * np.sin(theta_grid)

    points = np.stack([
        x_grid,
        y_grid,
        z_grid
    ], axis=-1)

    return points


def transform_sensor_object(points_sensor, R_sensor):
    """
    Transform geometry from sensor/body local frame to world/display frame.

    Steps:
        1. Rotate object in sensor/body frame using R_sensor.
        2. Convert rotated sensor/body coordinates to world/display coordinates.
    """

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
    """
    Read acceleration data from CSV.

    Supported column names:
        accx, accy, accz
        accel_x, accel_y, accel_z
        acc_x, acc_y, acc_z
        ax, ay, az
    """

    df = pd.read_csv(csv_path, encoding="utf-8-sig")

    # Normalize column names
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

    acc = np.array([
        row[col_x],
        row[col_y],
        row[col_z]
    ], dtype=float)

    print(f"Using acceleration columns: {col_x}, {col_y}, {col_z}")
    print(f"CSV row {row_index} acceleration data: {acc}")

    return acc


# ============================================================
# 5. Plotting
# ============================================================

def set_axes_equal(ax, limit=1.8):
    """
    Set equal scale for 3D axes.
    """

    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)

    try:
        ax.set_box_aspect([1, 1, 1])
    except Exception:
        pass


def plot_attitude(acc_current, row_index=None):
    """
    Plot cylinder attitude according to current acceleration vector.
    """

    acc_current = np.asarray(acc_current, dtype=float)

    if USE_NEGATIVE_ACCEL:
        acc_current = -acc_current

    # Reference direction in sensor/body frame.
    # When cylinder is vertical upward:
    #   accel_x approx +9800
    #   accel_y approx 0
    #   accel_z approx 0
    ref_vec_sensor = np.array([1.0, 0.0, 0.0], dtype=float)

    # Current acceleration direction in sensor/body frame.
    cur_vec_sensor = normalize(acc_current)

    # Compute attitude rotation:
    #   R_sensor @ ref_vec_sensor = cur_vec_sensor
    R_sensor = rotation_matrix_from_vectors(ref_vec_sensor, cur_vec_sensor)

    print("")
    print("========== ATTITUDE DEBUG ==========")
    print("Raw acceleration acc =", acc_current)
    print("Normalized acceleration direction =", cur_vec_sensor)
    print("Reference direction in sensor frame =", ref_vec_sensor)
    print("Rotation matrix R_sensor =")
    print(R_sensor)

    # Create cylinder in sensor/body frame
    cylinder_sensor = create_cylinder_along_sensor_x(
        length=CYLINDER_LENGTH,
        radius=CYLINDER_RADIUS,
        n_theta=140,
        n_length=110
    )

    # Transform cylinder to world/display frame
    cylinder_world = transform_sensor_object(cylinder_sensor, R_sensor)

    X = cylinder_world[:, :, 0]
    Y = cylinder_world[:, :, 1]
    Z = cylinder_world[:, :, 2]

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot_surface(
        X,
        Y,
        Z,
        color="#4C9FD6",
        alpha=1.0,
        linewidth=0,
        antialiased=True,
        shade=True
    )

    # Hide all axes and grid
    ax.set_axis_off()

    # Camera view
    ax.view_init(elev=22, azim=35)

    set_axes_equal(ax, limit=1.7)

    if row_index is None:
        ax.set_title("Cylinder Attitude")
    else:
        ax.set_title(f"Cylinder Attitude - CSV Row {row_index}")

    plt.tight_layout()
    plt.show()

    return R_sensor


# ============================================================
# 6. Main
# ============================================================

if __name__ == "__main__":
    acc = read_acc_from_csv(CSV_PATH, ROW_INDEX)
    plot_attitude(acc, row_index=ROW_INDEX)