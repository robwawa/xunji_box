#ifndef CHASSIS_INTERFACE_LEPU_PROTOCOL_H
#define CHASSIS_INTERFACE_LEPU_PROTOCOL_H

#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

namespace chassis_interface
{

// ============================================================
// 帧协议常量
// ============================================================
constexpr uint8_t FRAME_HEADER0 = 0xAA;
constexpr uint8_t FRAME_HEADER1 = 0x54;

// ============================================================
// 帧构建器: AA 54 <length> <ascii_payload> <checksum>
// ============================================================
inline uint8_t calcChecksum(const std::string& data)
{
  uint8_t result = static_cast<uint8_t>(data.size() & 0xFF);
  for (char c : data) result ^= static_cast<uint8_t>(c);
  return result;
}

inline std::vector<uint8_t> buildFrame(const std::string& cmd)
{
  std::vector<uint8_t> frame;
  frame.reserve(cmd.size() + 4);
  frame.push_back(FRAME_HEADER0);
  frame.push_back(FRAME_HEADER1);
  frame.push_back(static_cast<uint8_t>(cmd.size()));
  for (char c : cmd) frame.push_back(static_cast<uint8_t>(c));
  frame.push_back(calcChecksum(cmd));
  return frame;
}

// ============================================================
// 增量式帧解析器
// ============================================================
class FrameParser
{
public:
  std::vector<std::string> feed(const std::vector<uint8_t>& chunk);
private:
  std::vector<uint8_t> buffer_;
  std::string tryParseOne();
};

// ============================================================
// 消息解析器
// ============================================================
/**
 * 解析 nav_pose / nav:time_pose 消息
 * @param timestamp [out] 若有 nav:time_pose 格式则填入数据自带时间戳(秒)，否则保持原值
 * @return 解析成功返回 true
 */
bool parseNavPose(const std::string& msg, double& x, double& y, double& yaw,
                  double* timestamp = nullptr);
bool parseBaseVel(const std::string& msg, double& linear, double& angular);

/** 将配置模式映射为协议切换命令和精确确认响应。 */
bool getNavModeProtocol(const std::string& nav_mode, std::string& command,
                        std::string& expected_response);

/** 解析精确模式响应，返回 1/2/3；非模式响应返回 -1。 */
int parseNavModeResponse(const std::string& msg);

// ============================================================
// POSIX 串口 — 轻量级串口读写（无第三方依赖）
// ============================================================
class PosixSerial
{
public:
  PosixSerial() : fd_(-1) {}
  ~PosixSerial() { close(); }

  // 不可拷贝（持有文件描述符）
  PosixSerial(const PosixSerial&) = delete;
  PosixSerial& operator=(const PosixSerial&) = delete;

  bool open(const std::string& port, int baudrate);
  void close();
  bool isOpen() const { return fd_ >= 0; }

  /** 阻塞读取 (timeout 通过 select 实现)
   * @return >0=数据长度, 0=超时/可重试中断, -1=设备断联或致命错误
   */
  int read(uint8_t* buf, size_t max_len, int timeout_ms = 50);

  /** 阻塞写入 */
  int write(const uint8_t* data, size_t len);

private:
  int fd_;
};

// ============================================================
// 串口链路 — 后台线程读取 + 帧解析 + 消息回调
// ============================================================
using MessageCallback = std::function<void(const std::string&)>;
using ConnectionCallback = std::function<void(bool)>;

class LepuSerialLink
{
public:
  LepuSerialLink(const std::string& port, int baudrate, MessageCallback on_msg,
                 ConnectionCallback on_connection = ConnectionCallback(),
                 double reconnect_interval = 1.0);
  ~LepuSerialLink();

  // 不可拷贝（持有线程和文件描述符）
  LepuSerialLink(const LepuSerialLink&) = delete;
  LepuSerialLink& operator=(const LepuSerialLink&) = delete;

  bool open();
  void close();
  bool isOpen() const;
  bool sendCommand(const std::string& cmd);
  void requestReconnect();

  int reconnectCount() const { return reconnect_count_.load(); }
  int readErrorCount() const { return read_error_count_.load(); }
  int writeErrorCount() const { return write_error_count_.load(); }

private:
  void readLoop();
  void markDisconnected(const char* reason);
  bool waitForRetry();

  std::string port_;
  int baudrate_;
  MessageCallback on_message_;
  ConnectionCallback on_connection_;
  int reconnect_interval_ms_;

  PosixSerial serial_;
  FrameParser parser_;

  std::mutex write_mutex_;
  std::mutex wait_mutex_;
  std::condition_variable wait_cv_;
  std::thread read_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::atomic<bool> reconnect_requested_{false};
  std::atomic<int> reconnect_count_{0};
  std::atomic<int> read_error_count_{0};
  std::atomic<int> write_error_count_{0};
  bool ever_connected_ = false;  // 仅由 read_thread_ 访问
};

}  // namespace chassis_interface

#endif  // CHASSIS_INTERFACE_LEPU_PROTOCOL_H
