
```markdown
# IMU姿态解算与运动状态识别库
六轴惯性传感器信号滤波、姿态解算与运动状态识别算法库

## 项目简介
本仓库为 TDK ICM‑42670‑P 六轴惯性传感器开发的嵌入式算法库，包含两大核心模块：数字滤波器库与IMU姿态解算库。
实现原始传感器数据降噪、基于互补滤波的四元数姿态求解、ZUPT零偏动态补偿以及多运动状态识别。
项目支持 Python 离线算法仿真验证，同时提供 Qt 串口上位机，用于数据采集、波形可视化，算法内核使用标准C语言编写，方便移植到各类MCU。

## 仓库文件结构
```
├── filter_design_lib.h / filter_design_lib.c     # FIR / IIR 数字滤波器核心库
├── imu_api.h / imu_api.c                         # IMU姿态解算对外顶层API
├── micro_imu_lib.h / micro_imu_lib.c             # 底层姿态算法实现
├── Uart_Connect.cpp / .h                         # Qt串口通信模块
├── main.cpp                                      # Qt上位机程序入口
├── gesture_optimized.py                          # 离线数据集分析、滤波器调参脚本
├── main_realtime_optimized.py                    # Python实时算法验证脚本
├── main_realtime_with_trajectory.py              # 姿态轨迹可视化脚本
├── Temperature_Control_Platform.pro              # Qt工程文件
```

## 核心功能
### 1. 滤波器模块 filter_design_lib
- **FIR滤波器**：线性相位滤波，用于离线数据集预处理，输出无相位失真的参考信号，作为算法标定基准。
- **IIR滤波器**：计算开销低，资源占用小，用于嵌入式设备实时抑制传感器高频噪声。
- 开放参数配置接口，可自定义截止频率、滤波器阶数。

### 2. 姿态解算与状态机模块 imu_api
- 传感器数据融合，采用**互补滤波**求解四元数姿态，规避万向锁问题，输出欧拉角姿态信息。
- **ZUPT零速修正**：静止状态检测，动态校准陀螺仪零偏，有效抑制陀螺仪零偏带来的姿态漂移。
- 内置运动状态机：结合姿态角、角速度、运动强度、持续时间阈值，识别静置、移动、稳定持握、使用准备四种工作状态。

### 3. 算法验证与可视化工具
- Python脚本：离线分析传感器原始数据，对比FIR、IIR滤波输出效果，完成算法仿真调参。
- Qt图形上位机：串口接收IMU原始采样数据，实时绘制波形与姿态信息，方便算法调试。

## 编译运行环境
- 嵌入式算法内核：标准 C99，可直接移植至MCU
- 离线仿真：Python3，依赖 numpy、scipy、matplotlib
- 上位机：Qt6，QCustomPlot波形绘图组件

## 快速使用示例
### 嵌入式C库调用示例
```c
#include "imu_api.h"
#include "filter_design_lib.h"

// 初始化滤波器与姿态解算单元
filter_init();
imu_api_init();

while(1)
{
    // 读取加速度计、陀螺仪原始数据
    imu_api_feed_sample(acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z);
    // 姿态解算迭代
    imu_api_step();
    // 获取四元数、欧拉角姿态
    imu_get_attitude(&q, &euler_angle);
    // 获取识别得到的运动状态
    uint8_t motion_state = imu_get_motion_state();
}
```

### Python离线调试验证
运行 `gesture_optimized.py`，加载传感器离线数据集，完成滤波器参数调试与算法效果评估。

### Qt上位机
Qt Creator打开 `Temperature_Control_Platform.pro`，编译运行，串口连接硬件，实时查看数据波形与姿态。

## 算法亮点
1. 合理取舍FIR线性相位优势（离线做真值参考）与IIR低计算量优势（嵌入式实时运行），兼顾精度与设备算力限制。
2. 基于静止检测实现ZUPT零偏补偿方案，不需要额外硬件，改善长时间姿态解算漂移问题。
3. 模块化分层设计：滤波、姿态解算、状态机逻辑相互解耦，便于修改、裁剪与平台移植。

## 项目说明
本项目为实习期间开发，硬件：TDK ICM‑42670‑P 六轴IMU。
```


### 小建议
README顶部可以插入一张截图：滤波前后对比波形或者姿态轨迹图，面试官浏览观感更好。
如果你打算放到简历，直接把GitHub链接粘贴到项目/实习经历后方即可。
