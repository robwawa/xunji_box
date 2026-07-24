# chassis_interface

ROS1 Noetic 通用底盘接入层。桥接节点负责 ROS 接口、安全控制和插件加载；
公共驱动层负责九种里程计计算；具体底盘插件只负责通信协议。

## 架构

```text
move_base / 导航控制器
        │ /cmd_vel
        ▼
chassis_bridge_node（默认 50 Hz）
        │ ChassisDriver
        ├─ LepuDriver                 乐普兼容驱动，行为保持不变
        └─ CommonChassisDriver        新底盘推荐入口
             ├─ WheelOdometryDriver
             ├─ VelocityOdometryDriver
             ├─ DirectOdometryDriver
             ├─ DifferentialOdometryDriver
             ├─ DoubleDifferentialOdometryDriver
             ├─ SingleSteerOdometryDriver
             ├─ DoubleSteerOdometryDriver
             ├─ FourSteerOdometryDriver
             └─ GeneralKinematicsOdometryDriver
        │
        ├─ /odom
        ├─ /cmd_vel_feedback
        └─ /diagnostics
```

`ChassisDriver` 仍是 pluginlib 的统一接口。新驱动应优先继承与反馈类型匹配的
公共里程计父类，只实现：

```cpp
bool sendCommandToHardware(const ChassisCommand& cmd);
bool receiveStateFromHardware(HardwareFeedback& feedback);
```

底盘协议、串口/CAN/TCP、单位换算属于具体插件；积分、时间步保护、横向速度、
超时和诊断计数属于公共层。

## 目录

```text
include/chassis_interface/
├── chassis_driver.h
├── common_chassis_driver.h
├── chassis_bridge.h
└── drivers/lepu/lepu_protocol.h
src/
├── chassis_driver.cpp
├── common_chassis_driver.cpp
├── chassis_bridge.cpp
├── chassis_bridge_node.cpp
└── drivers/
    ├── lepu/
    └── template_driver.cpp
test/
├── kinematic_odometry.test
└── test_kinematic_odometry.cpp
config/
├── lepu_chassis.yaml
└── template_chassis.yaml
```

## 编译和测试

```bash
source /opt/ros/noetic/setup.bash
cd ~/test_ws
catkin_make --only-pkg-with-deps chassis_interface
catkin_make run_tests_chassis_interface
catkin_test_results
source devel/setup.bash
```

## 启动

```bash
# 乐普底盘
roslaunch chassis_interface chassis_bridge.launch chassis:=lepu

# 无硬件回环模板
roslaunch chassis_interface chassis_bridge.launch chassis:=template

# 自动加载 config/acme_chassis.yaml
roslaunch chassis_interface chassis_bridge.launch chassis:=acme

# 指定任意配置文件
roslaunch chassis_interface chassis_bridge.launch \
  config_path:=/absolute/path/acme_chassis.yaml
```

## ROS 接口

### 话题

| 名称 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 车体控制指令 |
| `/odom` | `nav_msgs/Odometry` | 发布 | 位姿和 `vx/vy/wz` |
| `/cmd_vel_feedback` | `geometry_msgs/Twist` | 发布 | 实际 `vx/vy/wz` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 发布 | 驱动健康状态 |

话题名均可通过 YAML 修改。桥接节点不发布 TF，避免与已有融合节点重复发布
`odom -> base_link`。

### 服务

节点名默认为 `chassis_bridge_node`：

| 名称 | 类型 | 说明 |
|---|---|---|
| `/chassis_bridge_node/enable` | `std_srvs/Trigger` | 使能 |
| `/chassis_bridge_node/disable` | `std_srvs/Trigger` | 禁能并停止 |
| `/chassis_bridge_node/reset_odom` | `std_srvs/Trigger` | 里程归零 |
| `/chassis_bridge_node/emergency_stop` | `std_srvs/Trigger` | 急停锁定 |

急停用于安全联锁。当前实现进入急停后请重启节点恢复，不要在实车调试脚本中
把急停当作普通停止指令。

### 控制和发布频率

`publish_rate` 默认是 `50.0 Hz`。桥接定时器每周期读取反馈、下发当前控制指令并
发布里程计，因此正常运行时控制下发频率也是 50 Hz。超过
`cmd_vel_timeout`（默认 0.2 s）未收到新指令时，下发零速度。

## 九种里程计模型

`HardwareFeedback` 中的轮速必须是轮地接触方向的有符号线速度，单位 m/s；
舵角单位 rad，车体坐标系 X 向前、Y 向左、逆时针为正。

| 父类 | 必填反馈 | 关键参数 | 典型底盘 |
|---|---|---|---|
| `WheelOdometryDriver` | 左右累计编码器 | `wheel_separation`、`wheel_radius`、`encoder_ticks_per_rev` | 编码器差速 |
| `VelocityOdometryDriver` | `linear_vel`、`angular_vel` | 标定和超时参数 | 下位机反馈车体速度 |
| `DirectOdometryDriver` | `x`、`y`、`yaw` | 标定和超时参数 | 下位机已有定位 |
| `DifferentialOdometryDriver` | 2 个轮速 | `wheel_separation` | 两轮差速 |
| `DoubleDifferentialOdometryDriver` | 4 个轮速、前后轮组角 | `wheelbase` | 前后差速轮组 |
| `SingleSteerOdometryDriver` | 1 个轮速、1 个舵角 | `wheelbase`、`steer_offset_y` | 单舵轮 |
| `DoubleSteerOdometryDriver` | 2 个轮速、2 个舵角 | `wheelbase`、`wheel_distance` | 前后双舵 |
| `FourSteerOdometryDriver` | 4 个轮速、4 个舵角 | 四轮安装坐标 | 四轮独立转向 |
| `GeneralKinematicsOdometryDriver` | N 个轮速、N 个方向角 | `num_wheels`、N 个安装坐标 | 通用多轮 |

完整、可复制的九种中文案例位于
`src/drivers/template_driver.cpp`。其中 `TemplateDriver` 本身是可运行的速度反馈
回环插件，不连接硬件也能验证桥接链路。

### 公共参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `odom_linear_scale` | 1.0 | 线速度/位移标定系数 |
| `odom_angular_scale` | 1.0 | 角速度/角度标定系数 |
| `max_feedback_dt` | 0.5 | 允许参与积分的最大反馈间隔 |
| `feedback_timeout` | 1.0 | 无反馈后判定断连，0 表示关闭 |
| `kinematic_damping` | 1e-9 | 多轮最小二乘阻尼 |

轮安装坐标示例：

```yaml
num_wheels: 4
wheel_positions_x: [0.50, 0.50, -0.50, -0.50]
wheel_positions_y: [0.30, -0.30, 0.30, -0.30]
```

数组顺序必须与 `wheel_speeds`、`steering_angles` 完全一致。



## 添加一个新底盘的完整流程

以下以四轮独立转向底盘 `AcmeDriver` 为例。

### 1. 确认原始反馈

先确认协议实际提供的是累计编码器、车体速度、绝对位姿，还是各轮线速度和舵角，
再从上表选择父类。不要在插件中重复实现公共积分。

### 2. 复制模板并实现协议扩展点

```bash
cp src/drivers/template_driver.cpp src/drivers/acme_driver.cpp
```

最小实现：

```cpp
#include <chassis_interface/common_chassis_driver.h>
#include <pluginlib/class_list_macros.h>

namespace chassis_interface
{
class AcmeDriver : public FourSteerOdometryDriver
{
protected:
  bool onConfigure(ros::NodeHandle& node) override
  {
    node.param<std::string>("port", port_, "/dev/ttyUSB0");
    return true;
  }

  bool onConnect() override
  {
    return protocol_.open(port_);
  }

  void onDisconnect() override
  {
    protocol_.close();
  }

  bool isHardwareConnected() const override
  {
    return protocol_.isOpen();
  }

  bool sendCommandToHardware(const ChassisCommand& cmd) override
  {
    return protocol_.sendBodyVelocity(cmd.linear_vel, cmd.angular_vel);
  }

  bool receiveStateFromHardware(HardwareFeedback& feedback) override
  {
    if (!protocol_.tryRead(
        feedback.wheel_speeds, feedback.steering_angles)) {
      return false;  // 没有新帧时立即返回，不能阻塞 50 Hz 主循环。
    }
    feedback.stamp = ros::Time::now();
    feedback.is_connected = true;
    return true;
  }

private:
  std::string port_;
  AcmeProtocol protocol_;
};
}  // namespace chassis_interface

PLUGINLIB_EXPORT_CLASS(
  chassis_interface::AcmeDriver, chassis_interface::ChassisDriver)
```

如硬件急停、清零或诊断需要专用协议，可按需覆盖
`sendEmergencyStopToHardware()`、`onResetOdometry()`、
`appendDiagnostics()`。

### 3. 加入编译

在 `CMakeLists.txt` 的共享库源文件列表中加入：

```cmake
add_library(chassis_interface
  # ...
  src/drivers/acme_driver.cpp
)
```

### 4. 注册 pluginlib 插件

在 `plugins/chassis_interface_plugin.xml` 中加入：

```xml
<class type="chassis_interface::AcmeDriver"
       base_class_type="chassis_interface::ChassisDriver">
  <description>Acme four-steer chassis</description>
</class>
```

类名、命名空间和 `PLUGINLIB_EXPORT_CLASS` 必须逐字一致。

### 5. 添加配置

创建 `config/acme_chassis.yaml`。ROS1 配置是扁平参数，由 launch 文件加载到
`chassis_bridge_node` 私有命名空间：

```yaml
driver_plugin: "chassis_interface::AcmeDriver"
port: "/dev/ttyUSB0"

odom_topic: "odom"
cmd_vel_topic: "cmd_vel"
odom_frame: "odom"
base_frame: "base_link"
publish_rate: 50.0
cmd_vel_timeout: 0.2
max_linear_speed: 0.5
max_angular_speed: 1.0

feedback_timeout: 1.0
max_feedback_dt: 0.5
odom_linear_scale: 1.0
odom_angular_scale: 1.0
wheel_positions_x: [0.50, 0.50, -0.50, -0.50]
wheel_positions_y: [0.30, -0.30, 0.30, -0.30]

cov_pose_xx: 0.05
cov_pose_yy: 0.05
cov_pose_yawyaw: 0.10
cov_twist_xx: 0.05
cov_twist_yy: 0.05
cov_twist_yawyaw: 0.05
```

### 6. 编译并检查插件

```bash
source /opt/ros/noetic/setup.bash
cd ~/test_ws
catkin_make --only-pkg-with-deps chassis_interface
source devel/setup.bash
rospack plugins --attrib=plugin chassis_interface
```

### 7. 无轮悬空验证

先断开动力或架空车轮：

```bash
roslaunch chassis_interface chassis_bridge.launch \
  chassis:=acme enable_ekf:=false

rosparam get /chassis_bridge_node/driver_plugin
rostopic hz /odom
rostopic echo -n 1 /odom
rostopic echo -n 1 /diagnostics
```

确认插件加载、反馈顺序、符号、单位和断连诊断都正确。

### 8. 低速闭环验证

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

依次验证前进、后退、左转、右转和停止；观察 `/cmd_vel_feedback` 与 `/odom`。
随后停止发布，确认约 0.2 s 后自动零速。

### 9. 里程归零与导航联调

```bash
rosservice call /chassis_bridge_node/reset_odom "{}"
```

最后再启用融合和导航，确认只有一个节点发布 `odom -> base_link` TF，检查协方差、
急停、拔线、重连和节点重启。

## 桥接参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `driver_plugin` | `chassis_interface::LepuDriver` | 插件类型 |
| `odom_topic` | `odom` | 里程计话题 |
| `cmd_vel_topic` | `cmd_vel` | 控制话题 |
| `publish_rate` | 50.0 | 读取、下发和发布频率 |
| `cmd_vel_timeout` | 0.2 | 控制超时 |
| `max_linear_speed` | 0.3 | 最大线速度 |
| `max_angular_speed` | 0.8 | 最大角速度 |
| `cov_pose_xx/yy/yawyaw` | 见 YAML | 位姿协方差 |
| `cov_twist_xx/yy/yawyaw` | 见 YAML | 速度协方差 |

乐普专属参数和协议行为仍由 `LepuDriver` 处理，现有
`config/lepu_chassis.yaml` 可以继续使用。

### 乐普 USB 断联与模式恢复

- 驱动只重开配置的稳定设备路径（默认 `/dev/lepu_chassis`），不会扫描或占用其他 `/dev/ttyACM*` 设备。
- EOF、致命读写错误或超过 `data_timeout` 未收到有效里程时关闭旧文件描述符，并按 `reconnect_interval` 自动重连。
- 每次连接后先发送零速和心跳，再设置并查询 `nav_mode`。只有收到精确的模式确认（`navi→model:1`、`mapping→model:2`、`remap→model:3`）并取得新里程数据后才恢复 READY 和里程发布。
- `nav_pose` 在恢复首帧重新建立会话锚点，避免通信模块复位导致累计里程跳变。

## 常见问题

- 插件加载失败：检查 XML 类名、导出宏、CMake 源文件以及是否重新
  `source devel/setup.bash`。
- 里程方向相反：优先修正协议层轮速符号或舵角零位，不要用协方差掩盖。
- 里程不增长：检查时间戳是否递增、反馈数组长度、`max_feedback_dt` 和诊断信息。
- 横移时 `y` 不变：确认选择了支持 `vy` 的模型，且桥接配置包含
  `cov_twist_yy`。
- 控制频率不对：检查 `publish_rate` 和 `rostopic hz /odom`，同时排除协议发送
  本身阻塞。
