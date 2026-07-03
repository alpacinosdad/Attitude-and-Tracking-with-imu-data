import sys

# 尝试将标准输出和标准错误输出的编码设置为 UTF-8
# 这样在控制台输出中文或特殊字符时，可以尽量避免乱码
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    # 如果当前 Python 环境不支持 reconfigure，直接忽略
    pass

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# CSV 文件路径，文件中应包含 IMU 姿态数据
CSV_PATH = "BAT_Heat_Log_Data_2026_06_22_16_19_45_imu_result.csv"

# 初始显示的 CSV 行索引
ROW_INDEX = 0

# 是否使用负加速度的开关
# 当前代码中该变量未被实际使用
USE_NEGATIVE_ACCEL = False

# 用于绘制的长方体尺寸
# 长方体代表传感器、电池包或需要显示姿态的物体
BOX_LENGTH_X = 20.0
BOX_WIDTH_Y = 5.0
BOX_HEIGHT_Z = 2.0

# 传感器坐标系到世界坐标系的固定转换矩阵
# 该矩阵定义了如何把传感器坐标转换成绘图用的世界坐标
#
# 根据矩阵可得：
# world_x = sensor_y
# world_y = -sensor_x
# world_z = sensor_z
SENSOR_TO_WORLD = np.array([
    [0.0, 1.0, 0.0],
    [-1.0, 0.0,  0.0],
    [0.0, 0.0,  1.0]
], dtype=float)


def load_attitude_df(csv_path):
    # 读取 CSV 文件
    # 使用 utf-8-sig 可以兼容带 BOM 的 UTF-8 CSV 文件
    df = pd.read_csv(csv_path, encoding="utf-8-sig")

    # 将所有列名转换为字符串，去除首尾空格，并统一转成小写
    # 这样后续判断 r11、qw 等列名时不受大小写和空格影响
    df.columns = [str(c).strip().lower() for c in df.columns]

    # 返回读取后的 DataFrame
    return df


def quaternion_to_rotation_matrix(qw, qx, qy, qz):
    # 将输入的四元数分量组成 numpy 数组
    # qw 为实部，qx/qy/qz 为虚部
    q = np.array([qw, qx, qy, qz], dtype=float)

    # 对四元数进行归一化
    # 归一化后才能正确表示旋转
    q = q / np.linalg.norm(q)

    # 拆分四元数分量
    w, x, y, z = q

    # 使用标准四元数转旋转矩阵公式
    # 返回 3x3 旋转矩阵
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ], dtype=float)


def get_rotation_matrix_from_row(row):
    # 定义旋转矩阵在 CSV 中可能出现的列名
    # 如果 CSV 已经包含 r11 到 r33，则可以直接组成旋转矩阵
    matrix_cols = [
        "r11", "r12", "r13",
        "r21", "r22", "r23",
        "r31", "r32", "r33"
    ]

    # 判断当前行中是否包含完整的旋转矩阵列
    if all(col in row.index for col in matrix_cols):
        # 直接从当前行读取 3x3 旋转矩阵
        return np.array([
            [row["r11"], row["r12"], row["r13"]],
            [row["r21"], row["r22"], row["r23"]],
            [row["r31"], row["r32"], row["r33"]],
        ], dtype=float)

    # 定义第一种四元数列名格式
    # qw 为实部，qx/qy/qz 为虚部
    quat_cols = ["qw", "qx", "qy", "qz"]

    # 判断当前行中是否包含 qw/qx/qy/qz 格式的四元数
    if all(col in row.index for col in quat_cols):
        # 将四元数转换为旋转矩阵
        return quaternion_to_rotation_matrix(
            row["qw"], row["qx"], row["qy"], row["qz"]
        )

    # 定义另一种四元数列名格式
    # 通常 q0 对应 qw，q1/q2/q3 对应 qx/qy/qz
    alt_quat_cols = ["q0", "q1", "q2", "q3"]

    # 判断当前行中是否包含 q0/q1/q2/q3 格式的四元数
    if all(col in row.index for col in alt_quat_cols):
        # 将四元数转换为旋转矩阵
        return quaternion_to_rotation_matrix(
            row["q0"], row["q1"], row["q2"], row["q3"]
        )

    # 如果既没有旋转矩阵，也没有支持的四元数格式，则抛出异常
    raise ValueError("CSV must contain R11..R33 or qw,qx,qy,qz (or q0..q3).")


def create_box_along_sensor_axes(length_x=BOX_LENGTH_X, width_y=BOX_WIDTH_Y, height_z=BOX_HEIGHT_Z):
    # 计算长方体在传感器坐标系下的边界
    # 长方体中心位于原点
    x0, x1 = -length_x / 2.0, length_x / 2.0
    y0, y1 = -width_y / 2.0, width_y / 2.0
    z0, z1 = -height_z / 2.0, height_z / 2.0

    # 用于保存长方体的六个面
    faces = []

    # 第 1 个面：z = z0，对应长方体底面
    faces.append(np.stack([np.array([[x0, x1], [x0, x1]]),
                            np.array([[y0, y0], [y1, y1]]),
                            np.array([[z0, z0], [z0, z0]])], axis=-1))

    # 第 2 个面：z = z1，对应长方体顶面
    faces.append(np.stack([np.array([[x0, x1], [x0, x1]]),
                            np.array([[y0, y0], [y1, y1]]),
                            np.array([[z1, z1], [z1, z1]])], axis=-1))

    # 第 3 个面：x = x0，对应长方体一侧面
    faces.append(np.stack([np.array([[x0, x0], [x0, x0]]),
                            np.array([[y0, y1], [y0, y1]]),
                            np.array([[z0, z0], [z1, z1]])], axis=-1))

    # 第 4 个面：x = x1，对应长方体另一侧面
    faces.append(np.stack([np.array([[x1, x1], [x1, x1]]),
                            np.array([[y0, y1], [y0, y1]]),
                            np.array([[z0, z0], [z1, z1]])], axis=-1))

    # 第 5 个面：y = y0，对应长方体一侧面
    faces.append(np.stack([np.array([[x0, x1], [x0, x1]]),
                            np.array([[y0, y0], [y0, y0]]),
                            np.array([[z0, z1], [z0, z1]])], axis=-1))

    # 第 6 个面：y = y1，对应长方体另一侧面
    faces.append(np.stack([np.array([[x0, x1], [x0, x1]]),
                            np.array([[y1, y1], [y1, y1]]),
                            np.array([[z0, z1], [z0, z1]])], axis=-1))

    # 返回长方体六个面的点坐标
    return faces


def transform_sensor_object(points_sensor, R_sensor):
    # 将输入点转换为 numpy 数组，确保后续可以进行矩阵运算
    points_sensor = np.asarray(points_sensor, dtype=float)

    # 将任意形状的点集展开为 N x 3
    # 其中每一行表示一个三维点
    flat = points_sensor.reshape(-1, 3)

    # 使用姿态旋转矩阵对物体点进行旋转
    # 因为点以行向量形式存储，所以右乘 R_sensor.T
    rotated = flat @ R_sensor.T

    # 将旋转后的点从传感器坐标系转换到世界坐标系
    # 注意：这里的 if False 永远不会执行前半部分
    # 实际执行的是 sensor_to_world(rotated)
    world = flat @ SENSOR_TO_WORLD.T if False else sensor_to_world(rotated)

    # 将结果恢复为输入点集原来的形状
    return world.reshape(points_sensor.shape)


def sensor_to_world(points_sensor):
    # 将输入点转换为 numpy 数组
    points_sensor = np.asarray(points_sensor, dtype=float)

    # 保存原始点集形状，方便转换后恢复
    shape = points_sensor.shape

    # 展开为 N x 3，便于统一进行矩阵乘法
    flat = points_sensor.reshape(-1, 3)

    # 使用 SENSOR_TO_WORLD 矩阵进行坐标系转换
    # 因为点以行向量形式存储，所以右乘矩阵转置
    world = flat @ SENSOR_TO_WORLD.T

    # 恢复为原始形状并返回
    return world.reshape(shape)


def set_axes_equal(ax, limit=22.0):
    # 设置 3D 坐标轴三个方向的显示范围
    # 这样可以让 x/y/z 三个方向的显示比例一致
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)

    # 设置 3D 坐标盒子的显示比例为 1:1:1
    # 某些旧版本 Matplotlib 可能不支持 set_box_aspect，因此使用 try 包裹
    try:
        ax.set_box_aspect([1, 1, 1])
    except Exception:
        pass


def plot_attitude_on_axes(ax, R_sensor, row_index=None, row=None):
    # 清空当前 3D 坐标轴内容，准备重新绘制
    ax.cla()

    # 在传感器坐标系下创建长方体的六个面
    faces_sensor = create_box_along_sensor_axes()

    # 默认将所有面设置为蓝色
    face_colors = ["#4C9FD6"] * len(faces_sensor)

    # 将第 4 个面设置为红色
    # 用于标识长方体的某一个方向，方便观察姿态变化
    face_colors[3] = "red"

    # 遍历长方体每一个面，并绘制到 3D 图中
    for face, color in zip(faces_sensor, face_colors):
        # 将当前面根据姿态旋转，并转换到世界坐标系
        face_world = transform_sensor_object(face, R_sensor)

        # 提取当前面的 X、Y、Z 坐标网格
        X = face_world[:, :, 0]
        Y = face_world[:, :, 1]
        Z = face_world[:, :, 2]

        # 绘制长方体表面
        ax.plot_surface(X, Y, Z, color=color, alpha=1.0, linewidth=0, antialiased=True, shade=True)

    # 定义传感器坐标系下的坐标轴端点
    # 第一个点是原点，后面三个点分别代表 X/Y/Z 轴方向
    axes_sensor = np.array([
        [0.0, 0.0, 0.0],
        [5.0, 0.0, 0.0],
        [0.0, 5.0, 0.0],
        [0.0, 0.0, 5.0]
    ], dtype=float)

    # 将坐标轴端点也按照当前姿态旋转，并转换到世界坐标系
    axes_world = transform_sensor_object(axes_sensor, R_sensor)

    # 取转换后的原点
    origin = axes_world[0]

    # 绘制传感器 X 轴，颜色为红色
    ax.plot([origin[0], axes_world[1, 0]], [origin[1], axes_world[1, 1]], [origin[2], axes_world[1, 2]], color="red", linewidth=3)

    # 绘制传感器 Y 轴，颜色为绿色
    ax.plot([origin[0], axes_world[2, 0]], [origin[1], axes_world[2, 1]], [origin[2], axes_world[2, 2]], color="green", linewidth=3)

    # 绘制传感器 Z 轴，颜色为蓝色
    ax.plot([origin[0], axes_world[3, 0]], [origin[1], axes_world[3, 1]], [origin[2], axes_world[3, 2]], color="blue", linewidth=3)

    # 隐藏坐标轴刻度、网格和边框，使显示更简洁
    ax.set_axis_off()

    # 设置 3D 视角
    # elev 表示仰角，azim 表示方位角
    ax.view_init(elev=22, azim=35)

    # 设置三个坐标轴等比例显示
    set_axes_equal(ax, limit=22.0)

    # 设置图像标题，显示当前数据行索引
    title = f"Box Attitude - Row {row_index}"

    # 如果当前行中包含 time_s 字段，则在标题中显示时间
    if row is not None and "time_s" in row.index:
        title += f"  Time {row['time_s']:.3f}s"

    # 应用标题
    ax.set_title(title)


if __name__ == "__main__":
    # 读取姿态 CSV 数据
    df = load_attitude_df(CSV_PATH)

    # 获取 CSV 数据总行数
    # 用于设置滑块的最大值
    num_rows = len(df)

    # 创建 Matplotlib 图像窗口
    fig = plt.figure(figsize=(10, 8))

    # 调整子图布局，为底部滑块留出空间
    plt.subplots_adjust(bottom=0.22)

    # 创建 3D 坐标轴
    ax = fig.add_subplot(111, projection="3d")

    # 创建滑块所在的坐标区域
    # 参数分别表示 left、bottom、width、height
    slider_ax = fig.add_axes([0.15, 0.08, 0.7, 0.04])

    # 创建滑块，用于选择 CSV 中的某一行数据
    row_slider = Slider(ax=slider_ax, label="Data Row", valmin=0, valmax=num_rows - 1,
                        valinit=ROW_INDEX, valstep=1)

    def on_slider(val):
        # 滑块变化时触发的回调函数

        # 将滑块值转换为整数，作为 DataFrame 的行索引
        idx = int(val)

        # 获取当前行数据
        row = df.iloc[idx]

        # 从当前行中提取旋转矩阵
        # 如果当前行是四元数格式，则会先转换为旋转矩阵
        R = get_rotation_matrix_from_row(row)

        # 根据当前旋转矩阵绘制长方体姿态
        plot_attitude_on_axes(ax, R, row_index=idx, row=row)

        # 通知 Matplotlib 画布进行刷新
        fig.canvas.draw_idle()

    # 将滑块变化事件绑定到 on_slider 回调函数
    row_slider.on_changed(on_slider)

    # 初始化显示 ROW_INDEX 对应的数据行姿态
    on_slider(ROW_INDEX)

    # 显示 Matplotlib 交互窗口
    plt.show()