# IMU姿态解算与运动状态识别库
六轴惯性传感器信号滤波、姿态解算与运动状态识别算法库

## 项目简介
本仓库为 **TDK ICM‑42670‑P** 六轴惯性传感器开发的嵌入式算法库，包含两大核心模块：数字滤波器库与IMU姿态解算库。
实现原始传感器数据降噪、基于互补滤波的四元数姿态求解、ZUPT零偏动态补偿以及多运动状态识别。
项目支持 Python 离线算法仿真验证，同时提供 Qt 串口上位机，用于数据采集、波形可视化。算法内核使用标准C99编写，可直接移植到各类MCU。

## 仓库文件结构
```bash
.
├── filter_design_lib.h / filter_design_lib.c     # FIR / IIR 数字滤波器核心库
├── imu_api.h / imu_api.c                         # IMU姿态解算对外顶层API
├── micro_imu_lib.h / micro_imu_lib.c             # 底层姿态算法实现
├── Uart_Connect.cpp / Uart_Connect.h             # Qt串口通信模块
├── main.cpp                                      # Qt上位机程序入口
├── gesture_optimized.py                          # 离线数据集分析、滤波器调参脚本
├── main_realtime_optimized.py                    # Python实时算法验证脚本
├── main_realtime_with_trajectory.py              # 姿态轨迹可视化脚本
└── Temperature_Control_Platform.pro              # Qt工程文件
