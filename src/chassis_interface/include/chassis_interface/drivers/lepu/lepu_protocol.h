#ifndef CHASSIS_INTERFACE_LEPU_PROTOCOL_H
#define CHASSIS_INTERFACE_LEPU_PROTOCOL_H

#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

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
bool parseNavPose(const std::string& msg, double& x, double& y, double& yaw);
bool parseBaseVel(const std::string& msg, double& linear, double& angular);
bool parseWheelEncoders(const std::string& msg, int32_t& left, int32_t& right);

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

  /** 阻塞读取 (timeout 通过 select 实现) */
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

class LepuSerialLink
{
public:
  LepuSerialLink(const std::string& port, int baudrate, MessageCallback on_msg);
  ~LepuSerialLink();

  // 不可拷贝（持有线程和文件描述符）
  LepuSerialLink(const LepuSerialLink&) = delete;
  LepuSerialLink& operator=(const LepuSerialLink&) = delete;

  bool open();
  void close();
  bool isOpen() const;
  void sendCommand(const std::string& cmd);

private:
  void readLoop();

  std::string port_;
  int baudrate_;
  MessageCallback on_message_;

  PosixSerial serial_;
  FrameParser parser_;

  std::mutex write_mutex_;
  std::thread read_thread_;
  std::atomic<bool> running_{false};
};

}  // namespace chassis_interface

#endif  // CHASSIS_INTERFACE_LEPU_PROTOCOL_H
