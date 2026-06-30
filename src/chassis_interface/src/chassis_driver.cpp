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

}  // namespace chassis_interface
