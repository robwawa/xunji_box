#ifndef CHASSIS_INTERFACE_CHASSIS_DRIVER_H
#define CHASSIS_INTERFACE_CHASSIS_DRIVER_H

#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

namespace chassis_interface
{

// ============================================================
// 底盘状态 — 从驱动层上报的标准数据结构
// ============================================================
struct ChassisState
{
  double x = 0.0;               // 里程计位置 X (m)
  double y = 0.0;               // 里程计位置 Y (m)
  double yaw = 0.0;             // 里程计偏航角 (rad)
  double linear_vel = 0.0;      // 线速度 (m/s)
  double angular_vel = 0.0;     // 角速度 (rad/s)
  int32_t left_encoder = 0;     // 左轮编码器原始值
  int32_t right_encoder = 0;    // 右轮编码器原始值
  ros::Time stamp;              // 状态时间戳
  bool is_connected = false;    // 硬件连接状态
  int error_code = 0;           // 0=正常, 非0=错误码
  std::string status_msg;       // 状态描述
};

// ============================================================
// 底盘指令 — 导航层下发到底盘的标准数据结构
// ============================================================
struct ChassisCommand
{
  double linear_vel = 0.0;      // 目标线速度 (m/s)
  double angular_vel = 0.0;     // 目标角速度 (rad/s)
  ros::Time stamp;              // 指令时间戳
};

// ============================================================
// 生命周期状态
// ============================================================
enum class LifecycleState
{
  UNCONFIGURED,   // 初始
  CONFIGURING,    // 加载驱动插件中
  READY,          // 驱动就绪，等待使能
  RUNNING,        // 正常运行
  ERROR,          // 故障（可恢复）
  EMERGENCY       // 急停（需手动复位）
};

// ============================================================
// ChassisDriver — 底盘驱动抽象基类
//
// 所有底盘驱动必须继承此类并实现纯虚函数。
// 使用 pluginlib 加载，参见 chassis_interface_plugin.xml
// ============================================================
class ChassisDriver
{
public:
  ChassisDriver() = default;
  virtual ~ChassisDriver() = default;

  /** 初始化并连接底盘硬件
   * @param nh 私有节点句柄，用于读取驱动专属参数
   * @return true=成功, false=失败
   */
  virtual bool connect(ros::NodeHandle& nh) = 0;

  /** 断开底盘连接，安全停止 */
  virtual void disconnect() = 0;

  /** 读取底盘当前状态（编码器、速度、位姿等）
   * @return 当前底盘状态快照
   */
  virtual ChassisState readState() = 0;

  /** 下发运动指令到底盘
   * @param cmd 目标线速度/角速度
   */
  virtual void writeCommand(const ChassisCommand& cmd) = 0;

  /** 紧急停止 — 立即停止所有运动，发送停止指令 */
  virtual void emergencyStop() = 0;

  // ---- 可选重载 ----

  /** 重置里程计归零 */
  virtual void resetOdom();

  /** 填充诊断信息
   * @param stat 诊断状态包装器
   */
  virtual void getDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

  /** 获取驱动名称 */
  virtual std::string getDriverName() const;

protected:
  /** 差速模型里程计积分（中点法）
   * 调用者负责持有 odom_x_/odom_y_/odom_yaw_ 的锁
   */
  static void integrateMotion(double& x, double& y, double& yaw,
                              double& v, double& w,
                              double delta_center, double delta_yaw, double dt);

  /** 角度归一化到 [-π, π] */
  static double normalizeAngle(double angle);
};

}  // namespace chassis_interface

// pluginlib 导出宏 — 子类在 .cpp 中使用:
//   PLUGINLIB_EXPORT_CLASS(chassis_interface::LepuDriver, chassis_interface::ChassisDriver)

#endif  // CHASSIS_INTERFACE_CHASSIS_DRIVER_H
