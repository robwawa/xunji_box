#include <chassis_interface/chassis_driver.h>

namespace chassis_interface
{

// ============================================================
// ChassisDriver 默认实现
// ============================================================
// 这些函数提供合理的默认行为，子类可以重载

void ChassisDriver::resetOdom()
{
  // 默认空实现 — 如果底盘不支持里程计重置，什么都不做
}

void ChassisDriver::getDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Driver OK (no diagnostics)");
}

std::string ChassisDriver::getDriverName() const
{
  return "unknown";
}
// ============================================================
// 计算运动积分，根据当前状态和增量运动计算积分运动
// ============================================================
void ChassisDriver::integrateMotion(double& x, double& y, double& yaw,
                                    double& v, double& w,
                                    double delta_center, double delta_yaw, double dt)
{
  if (std::abs(delta_center) < 1e-9 && std::abs(delta_yaw) < 1e-9)
  {
    v = 0.0;
    w = 0.0;
    return;
  }

  double mid_yaw = yaw + 0.5 * delta_yaw;
  x   += delta_center * std::cos(mid_yaw);
  y   += delta_center * std::sin(mid_yaw);
  yaw  = normalizeAngle(yaw + delta_yaw);

  dt = std::max(dt, 1e-6);
  v = delta_center / dt;
  w = delta_yaw / dt;
}
// ============================================================
// 角度归一化
// ============================================================
double ChassisDriver::normalizeAngle(double angle)
{
  while (angle > M_PI)  angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

}  // namespace chassis_interface
