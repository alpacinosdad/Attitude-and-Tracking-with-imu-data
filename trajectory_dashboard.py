import numpy as np
import matplotlib.pyplot as plt

from gesture_optimized import RealtimeBoxViewer


class RealtimeTrajectoryDashboard:
    """
    实时大图显示：
        左上：完整重心轨迹
        右上：实时姿态圆柱体
        下方：加速度、速度、位移曲线
    """

    def __init__(self):
        self.fig = plt.figure(figsize=(18, 13))

        gs = self.fig.add_gridspec(
            5,
            4,
            height_ratios=[1.1, 1.1, 0.45, 0.45, 0.45],
            hspace=0.55,
            wspace=0.3,
            top=0.95,
            bottom=0.10,
        )

        self.ax_traj = self.fig.add_subplot(gs[0:2, 0:2], projection="3d")
        self.ax_orient = self.fig.add_subplot(gs[0:2, 2:], projection="3d")
        self.ax_acc = self.fig.add_subplot(gs[2, :])
        self.ax_vel = self.fig.add_subplot(gs[3, :], sharex=self.ax_acc)
        self.ax_disp = self.fig.add_subplot(gs[4, :], sharex=self.ax_acc)

        self.orientation_viewer = RealtimeBoxViewer(self.ax_orient)

        self.traj_line, = self.ax_traj.plot([], [], [], color="tab:blue", lw=1.5, alpha=0.85, label="center trajectory")
        self.current_point, = self.ax_traj.plot([], [], [], "o", color="red", markersize=7, label="current center")

        self.ax_traj.set_title("Center Trajectory")
        self.ax_traj.set_xlabel("X")
        self.ax_traj.set_ylabel("Y")
        self.ax_traj.set_zlabel("Z")
        self.ax_traj.legend(loc="upper left")

        self.acc_lines = [
            self.ax_acc.plot([], [], label="ax_world", color="C0", alpha=0.85)[0],
            self.ax_acc.plot([], [], label="ay_world", color="C1", alpha=0.85)[0],
            self.ax_acc.plot([], [], label="az_world", color="C2", alpha=0.85)[0],
        ]
        self.ax_acc.set_title("Linear Acceleration Used")
        self.ax_acc.set_ylabel("accel")
        self.ax_acc.grid(True)
        self.ax_acc.legend(loc="upper right", fontsize="small", ncol=3)

        self.vel_lines = [
            self.ax_vel.plot([], [], label="vx", color="C0", alpha=0.85)[0],
            self.ax_vel.plot([], [], label="vy", color="C1", alpha=0.85)[0],
            self.ax_vel.plot([], [], label="vz", color="C2", alpha=0.85)[0],
        ]
        self.ax_vel.set_title("Velocity Used")
        self.ax_vel.set_ylabel("velocity")
        self.ax_vel.grid(True)
        self.ax_vel.legend(loc="upper right", fontsize="small", ncol=3)

        self.disp_lines = [
            self.ax_disp.plot([], [], label="dx", color="C0", alpha=0.85)[0],
            self.ax_disp.plot([], [], label="dy", color="C1", alpha=0.85)[0],
            self.ax_disp.plot([], [], label="dz", color="C2", alpha=0.85)[0],
        ]
        self.ax_disp.set_title("Displacement Used")
        self.ax_disp.set_xlabel("time_s (s)")
        self.ax_disp.set_ylabel("displacement")
        self.ax_disp.grid(True)
        self.ax_disp.legend(loc="upper right", fontsize="small", ncol=3)

        plt.setp(self.ax_acc.get_xticklabels(), visible=False)
        plt.setp(self.ax_vel.get_xticklabels(), visible=False)

        self.info_text = self.fig.text(
            0.15,
            0.035,
            "",
            fontsize=10,
            family="monospace",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.9, edgecolor="gray"),
        )

        self.traj_limit_initialized = False
        self.curve_update_count = 0

    def set_initial_view_by_accel_z(self, acc_z):
        self.orientation_viewer.set_initial_view_by_accel_z(acc_z)

    @staticmethod
    def _set_equal_3d(ax, data):
        data = np.asarray(data, dtype=float)
        if data.ndim != 2 or data.shape[1] != 3 or len(data) == 0:
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)
            return

        finite = np.isfinite(data).all(axis=1)
        if not np.any(finite):
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)
            return

        data = data[finite]
        xs, ys, zs = data.T

        max_range = np.max([
            xs.max() - xs.min(),
            ys.max() - ys.min(),
            zs.max() - zs.min(),
        ]) / 2.0

        if max_range < 1e-6:
            max_range = 0.1

        mid_x = (xs.max() + xs.min()) / 2.0
        mid_y = (ys.max() + ys.min()) / 2.0
        mid_z = (zs.max() + zs.min()) / 2.0

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        try:
            ax.set_box_aspect([1, 1, 1])
        except Exception:
            pass

    @staticmethod
    def _autoscale_2d(ax, t, y):
        if len(t) == 0:
            return
        ax.set_xlim(t[0], max(t[-1], t[0] + 1e-3))
        finite = np.isfinite(y)
        if not np.any(finite):
            ax.set_ylim(-1, 1)
            return
        ymin = np.min(y[finite])
        ymax = np.max(y[finite])
        if abs(ymax - ymin) < 1e-9:
            ymin -= 1.0
            ymax += 1.0
        pad = 0.1 * (ymax - ymin)
        ax.set_ylim(ymin - pad, ymax + pad)

    def update(self, time_s, R_display, histories, is_stationary):
        t, acc, vel, disp, static_mask = histories

        # 右上角姿态圆柱体
        self.orientation_viewer.update(R_display, time_s=time_s)

        if len(t) == 0:
            return

        # 左上完整重心轨迹
        self.traj_line.set_data(disp[:, 0], disp[:, 1])
        self.traj_line.set_3d_properties(disp[:, 2])

        self.current_point.set_data([disp[-1, 0]], [disp[-1, 1]])
        self.current_point.set_3d_properties([disp[-1, 2]])

        self._set_equal_3d(self.ax_traj, disp)

        # 下方曲线：实时全历史
        for k in range(3):
            self.acc_lines[k].set_data(t, acc[:, k])
            self.vel_lines[k].set_data(t, vel[:, k])
            self.disp_lines[k].set_data(t, disp[:, k])

        # 曲线自动缩放。为减小开销，可以不是每帧都重新缩放。
        self.curve_update_count += 1
        if self.curve_update_count % 5 == 0:
            self._autoscale_2d(self.ax_acc, t, acc)
            self._autoscale_2d(self.ax_vel, t, vel)
            self._autoscale_2d(self.ax_disp, t, disp)

        self.info_text.set_text(
            f"Time: {time_s:.3f}s | Samples: {len(t)} | Static: {bool(is_stationary)}\n"
            f"Position: X={disp[-1,0]:+.6f}, Y={disp[-1,1]:+.6f}, Z={disp[-1,2]:+.6f}\n"
            f"Velocity: X={vel[-1,0]:+.6f}, Y={vel[-1,1]:+.6f}, Z={vel[-1,2]:+.6f}"
        )

        self.fig.canvas.draw_idle()
        plt.pause(0.001)
