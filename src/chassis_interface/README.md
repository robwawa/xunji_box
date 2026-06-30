# chassis_interface

通用底盘接入层 — 基于 pluginlib 插件架构的 ROS1 C++ 功能包。

任何底盘只需实现驱动插件即可接入导航控制系统，无需修改桥接层代码。

## 架构

```
导航层 (move_base / 自研)
   │  cmd_vel              odom + tf
   ▼                          ▲
┌──────────────────────────────┐
│  chassis_bridge_node (C++)   │  ← 通用桥接节点
│  · 生命周期状态机             │
│  · 诊断 / 安全限幅            │
│  · pluginlib 动态加载驱动     │
└──────────┬───────────────────┘
           │ 纯虚接口
┌──────────▼───────────────────┐
│  ChassisDriver (抽象基类)     │
│  connect / readState         │
│  writeCommand / emergencyStop│
└──────────┬───────────────────┘
           │ 实现
    ┌──────┴──────┐
    │ LepuDriver  │  TemplateDriver  ...
    │ 乐普串口协议 │  新底盘模板
    └─────────────┘
```

## 目录结构

```
chassis_interface/
├── include/chassis_interface/
│   ├── chassis_driver.h              # 抽象基类 + 数据结构
│   ├── chassis_bridge.h              # 桥接节点声明
│   └── drivers/lepu/
│       └── lepu_protocol.h           # 乐普 AA 54 帧协议
├── src/
│   ├── chassis_driver.cpp            # 基类默认实现
│   ├── chassis_bridge.cpp            # 桥接节点 (状态机/ROS接口)
│   ├── chassis_bridge_node.cpp       # main() 入口
│   └── drivers/
│       ├── lepu/
│       │   ├── lepu_driver.cpp       # 乐普驱动插件
│       │   └── lepu_protocol.cpp     # 乐普协议实现 (POSIX 串口)
│       └── template_driver.cpp       # 新底盘开发模板
├── config/
│   ├── lepu_chassis.yaml
│   └── template_chassis.yaml
├── launch/
│   ├── lepu_chassis.launch
│   └── template_chassis.launch
├── chassis_interface_plugin.xml
├── CMakeLists.txt
└── package.xml
```

## 编译

```bash
# 依赖
sudo apt install ros-noetic-diagnostic-updater

# 编译 (零外部串口依赖，使用 POSIX termios)
cd your_workspace
catkin_make
source devel/setup.bash
```

## 快速使用

```bash
# 乐普底盘
roslaunch chassis_interface lepu_chassis.launch

# 指定串口
roslaunch chassis_interface lepu_chassis.launch port:=/dev/ttyUSB0

# 模板驱动 (测试插件机制)
roslaunch chassis_interface template_chassis.launch
```

## ROS API

### 话题

| 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/odom` (可配) | `nav_msgs/Odometry` | 发布 | 里程计 (含协方差) |
| `/cmd_vel` (可配) | `geometry_msgs/Twist` | 订阅 | 速度指令 (带安全限幅+超时停止) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 发布 | 驱动健康状态 |
| TF | `odom → base_link` | 发布 | 坐标变换 |

### 服务

| 名称 | 类型 | 说明 |
|------|------|------|
| `~enable` | `std_srvs/Trigger` | 使能底盘 |
| `~disable` | `std_srvs/Trigger` | 禁能底盘 (发送零速) |
| `~reset_odom` | `std_srvs/Trigger` | 重置里程计归零 |
| `~emergency_stop` | `std_srvs/Trigger` | 急停 (需手动 reset) |

### 参数

**桥接节点公共参数：**

| 名称 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `~driver_plugin` | string | `chassis_interface::LepuDriver` | 驱动插件类名 |
| `~odom_topic` | string | `odom` | 里程计话题名 |
| `~cmd_vel_topic` | string | `cmd_vel` | 速度指令话题名 |
| `~odom_frame` | string | `odom` | 里程计坐标系 |
| `~base_frame` | string | `base_link` | 基座坐标系 |
| `~odom_queue_size` | int | 50 | odom 发布队列 |
| `~cmd_vel_queue_size` | int | 1 | cmd_vel 订阅队列 |
| `~max_linear_speed` | double | 0.3 | 安全限幅 (m/s) |
| `~max_angular_speed` | double | 0.8 | 安全限幅 (rad/s) |
| `~cmd_vel_timeout` | double | 0.2 | 指令超时自动停止 (s) |
| `~publish_rate` | double | 50.0 | 发布频率 (Hz) |
| `~diag_rate` | double | 1.0 | 诊断频率 (Hz) |

**里程计协方差参数（影响 EKF/UKF 融合权重）：**

| 名称 | 默认值 | 说明 |
|------|--------|------|
| `~cov_pose_xx` | 0.05 | 位置 X 方差 (m²) |
| `~cov_pose_yy` | 0.05 | 位置 Y 方差 (m²) |
| `~cov_pose_yawyaw` | 0.1 | 偏航角方差 (rad²) |
| `~cov_twist_xx` | 0.05 | 线速度方差 (m²/s²) |
| `~cov_twist_yawyaw` | 0.05 | 角速度方差 (rad²/s²) |

**乐普驱动专属参数：**

| 名称 | 默认值 | 说明 |
|------|--------|------|
| `~port` | `/dev/ttyACM0` | 串口设备 |
| `~baudrate` | 115200 | 波特率 |
| `~wheel_separation` | 0.242 | 两轮间距 (m) |
| `~wheel_radius` | 0.0705 | 轮子半径 (m) |
| `~encoder_ticks_per_rev` | 16384 | 编码器每圈脉冲 |
| `~use_encoder_odom` | true | 使用编码器里程计 (false=nav_pose) |
| `~odom_linear_scale` | 1.0 | 线速度标定系数 |
| `~odom_angular_scale` | 1.0 | 角速度标定系数 |
| `~encoder_dt_max` | 0.5 | 编码器最大有效间隔 (s) |
| `~encoder_dt_fallback` | 0.05 | 编码器兜底间隔 (s) |

### 生命周期

```
UNCONFIGURED → CONFIGURING → READY → RUNNING → ERROR → READY
                                    ↑         ↓
                                    ←←←←←←←←←←
                                    ↓
                                EMERGENCY (需 ~enable 恢复)
```

- **RUNNING**: 正常接收 cmd_vel 并执行
- **ERROR**: 连接断开 (自动尝试恢复)
- **EMERGENCY**: 急停锁定

## 里程计模式

LepuDriver 支持三种里程计来源，通过 `~use_encoder_odom` 参数切换：

| 模式 | 说明 | 适用场景 |
|------|------|---------|
| 编码器 (`use_encoder_odom=true`) | 轮式编码器 → 差速模型积分 | 常规使用 |
| base_vel (`use_encoder_odom=true`) | 底盘上报的线/角速度 | 速度模式 |
| nav_pose (`use_encoder_odom=false`) | SLAM 全局位姿 → 相对变换 | 编码器不可用时 |

## 接入新底盘

### Step 1: 复制模板

```bash
cp src/drivers/template_driver.cpp src/drivers/your_driver.cpp
```

模板已包含完整的差速模型里程计、POSIX 串口骨架、诊断上报代码，开发者只需填充标注了 `TODO` 的硬件通信和协议转换部分。

### Step 2: 实现接口

```cpp
class YourDriver : public ChassisDriver
{
    bool connect(ros::NodeHandle& nh) override
    {
        // 读取参数: nh.param<T>("key", val, default)
        // 打开串口/CAN/以太网
        return true;
    }

    ChassisState readState() override
    {
        ChassisState state;
        // 1. 从底盘读取编码器
        // 2. 差速模型积分 (模板中 integrateMotion 可直接复用)
        // 3. 填充 state.x, state.y, state.yaw, state.linear_vel...
        state.stamp = ros::Time::now();
        state.is_connected = true;
        return state;
    }

    void writeCommand(const ChassisCommand& cmd) override
    {
        // cmd.linear_vel  → 目标线速度 (m/s)
        // cmd.angular_vel → 目标角速度 (rad/s)
        // 转为底盘协议帧发送
    }

    void emergencyStop() override
    {
        // 发送停止指令 + 清零内部速度
    }

    std::string getDriverName() const override { return "your_chassis"; }
};

PLUGINLIB_EXPORT_CLASS(chassis_interface::YourDriver, chassis_interface::ChassisDriver)
```

### Step 3: 注册插件

在 `chassis_interface_plugin.xml` 添加：

```xml
<class type="chassis_interface::YourDriver"
       base_class_type="chassis_interface::ChassisDriver">
  <description>Your chassis driver</description>
</class>
```

### Step 4: 添加编译

在 `CMakeLists.txt` 的 `add_library(chassis_interface ...)` 中添加：

```cmake
src/drivers/your_driver.cpp
```

### Step 5: 配置参数

复制 `config/template_chassis.yaml` → 修改驱动专属参数和协方差。

### Step 6: 编译运行

```bash
catkin_make
roslaunch chassis_interface template_chassis.launch
```

## 已支持的底盘

| 驱动 | 协议 | 接口 | 状态 |
|------|------|------|------|
| LepuDriver | SLAM 3.0 (AA 54 帧, USB-RS232) | `app_vel[linear,angular]` | ✅ |
| TemplateDriver | — | 完整开发模板 (含差速模型) | 模板 |

## 设计要点

- **零外部串口依赖**：使用 POSIX termios，不需 `ros-noetic-serial`
- **线程安全**：`boost::shared_mutex` 保护里程计状态，`std::mutex` 保护硬件句柄
- **安全限幅**：`boost::algorithm::clamp` 限制 cmd_vel 幅度
- **超时停止**：cmd_vel 超过 `~cmd_vel_timeout` 未更新则自动发送零速
- **参数化协方差**：5 个协方差参数支持按底盘精度标定

## 许可证

MIT
