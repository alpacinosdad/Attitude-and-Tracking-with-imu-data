"""

2.去重力
├─ 获取用于积分的线性加速度 accel_raw_linear
│
├─ 判断加速度来源
│   │
│   ├─ 如果 USE_AACCEL_AS_WORLD_LINEAR == True
│   │   └─ get_world_linear_accel_from_aaccel
│   │       ├─ 读取世界系总加速度
│   │       │   └─ aaccel = [aax, aay, aaz]
│   │       │
│   │       ├─ 去除世界系重力
│   │       │   └─ linear_world = [aax, aay, aaz - GRAVITY]
│   │       │
│   │       └─ 输出世界系线性加速度
│   │           └─ accel_raw_linear = linear_world
│   │
│   └─ 如果 USE_AACCEL_AS_WORLD_LINEAR == False
│       └─ get_body_linear_accel_from_lax
│           ├─ 读取 C 端输出的 body 系线性加速度
│           │   └─ linear_body = [lax, lay, laz]
│
    
"""