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
│   └── chassis_bridge.launch       # 通用启动文件
├── plugins/
│   └── chassis_interface_plugin.xml # 插件注册
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
# 乐普底盘 (默认加载 config/lepu_chassis.yaml)
roslaunch chassis_interface chassis_bridge.launch

# 切换模板驱动
roslaunch chassis_interface chassis_bridge.launch chassis:=template

# 自定义底盘 (自动加载 config/my_robot_chassis.yaml)
roslaunch chassis_interface chassis_bridge.launch chassis:=my_robot

# 直接指定配置文件路径
roslaunch chassis_interface chassis_bridge.launch config_path:=/tmp/special.yaml

# 覆盖参数
roslaunch chassis_interface chassis_bridge.launch chassis:=lepu \
    port:=/dev/ttyUSB0 odom_topic:=/my_odom
```

## ROS API

### 话题

| 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/odom` (可配) | `nav_msgs/Odometry` | 发布 | 里程计 (含协方差) |
| `/cmd_vel` (可配) | `geometry_msgs/Twist` | 订阅 | 速度指令 (带安全限幅+超时停止) |
| `/cmd_vel_feedback` | `geometry_msgs/Twist` | 发布 | 实际速度反馈 (MPC闭环控制) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 发布 | 驱动健康状态 |

> **TF 说明**：chassis_bridge_node 不发布 TF，由外部节点（xrobot_driver_odom_fusion / robot_localization）基于 `/odom` 数据融合后发布。

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
| `~port` | `/dev/lepu_chassis` | 串口设备 (udev 持久化命名) |
| `~baudrate` | 115200 | 波特率 |
| `~enable_pose_report` | true | 开启 nav_pose 自动上报 |
| `~odom_source` | `"nav_pose"` | 里程计来源: `base_vel`(速度积分) 或 `nav_pose`(绝对位姿) |
| `~nav_mode` | `"mapping"` | 导航模式: `mapping`(建图) `navi`(导航) `remap`(增量建图) |
| `~odom_linear_scale` | 1.0 | 线速度标定系数 |
| `~odom_angular_scale` | 1.0 | 角速度标定系数 |
| `~dt_max` | 0.5 | 最大有效时间间隔 (s) |
| `~vel_lpf_alpha` | 0.3 | 速度低通滤波系数 (0~1, 越小越平滑) |
| `~max_vel_change` | 0.5 | 最大速度变化率 (m/s², 防止SLAM跳变) |
| `~reconnect_interval` | 1.0 | USB断联后通过稳定设备别名重试的间隔 (s) |
| `~data_timeout` | 1.0 | 无有效里程计数据后强制重连的超时 (s) |

### USB 断联与模式恢复

- 驱动只重开配置的稳定设备路径（默认 `/dev/lepu_chassis`），不会扫描或占用其他 `/dev/ttyACM*` 设备。
- EOF、致命读写错误或里程数据超时会关闭旧文件描述符，并由后台线程自动重连。
- 每次连接后先发送零速和心跳，再设置并查询 `nav_mode`。只有收到精确的模式确认（`navi→model:1`、`mapping→model:2`、`remap→model:3`）并取得新里程数据后才恢复 READY 和里程发布。
- `nav_pose` 在恢复首帧重新建立会话锚点，避免通信模块复位导致累计里程跳变。

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

LepuDriver 支持两种里程计来源，通过 `~odom_source` 参数切换：

| 模式 | 协议 | 算法 | 特点 |
|------|------|------|------|
| `base_vel` | `base_vel[v,w]` (§9) | 速度×dt → 中点法积分 | 平滑连续，累积误差 |
| `nav_pose` (默认) | `nav:pose[x,y,θ]` / `nav:time_pose[x,y,θ,ts]` (§27/§37) | 相对 origin 旋转变换 + 位置差分 + 低通滤波 | 绝对值，无累积误差 |

> **nav_pose 速度计算**：仅用带时间戳的 `nav:time_pose` 计算速度（避免同批次 `nav:pose` 导致的零速衰减），位移投影到车头方向确定正负号。

选择逻辑：
```cpp
// 启动时读取一次，运行时不切换
if (odom_source_ == "base_vel")
    parseBaseVel(msg) → integrateMotion(v*dt, w*dt)
else if (odom_source_ == "nav_pose")
    parseNavPose(msg) → origin相对变换
```

注：官方协议 `core_data`(§4) 和 `wheel_status`(§5) 不含编码器字段，因此无编码器里程计模式。`integrateMotion` 等核心算法保留在驱动中供未来硬件扩展。

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
roslaunch chassis_interface chassis_bridge.launch chassis:=your_chassis
```

## 已支持的底盘

| 驱动 | 协议 | 接口 | 状态 |
|------|------|------|------|
| LepuDriver | SLAM 3.0 (AA 54 帧, USB-RS232) | `app_vel[linear,angular]` | ✅ |
| TemplateDriver | — | 完整开发模板 (含差速模型) | 模板 |

## 设计要点

- **零外部串口依赖**：使用 POSIX termios，不需 `ros-noetic-serial`
- **协议合规**：严格按官方 `slam_api.txt` 协议文档实现，逐字段验证
- **线程安全**：`boost::shared_mutex` 保护里程计状态，`std::mutex` 保护硬件句柄
- **安全限幅**：`boost::algorithm::clamp` 限制 cmd_vel 幅度
- **超时停止**：cmd_vel 超过 `~cmd_vel_timeout` 未更新则自动发送零速
- **参数化协方差**：5 个协方差参数支持按底盘精度标定
- **里程计算法保留**：`integrateMotion` / `encoderDelta` 等核心算法保留供硬件扩展

## 许可证

MIT
