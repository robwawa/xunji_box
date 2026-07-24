#include <chassis_interface/drivers/lepu/lepu_protocol.h>
#include <ros/console.h>
#include <regex>
#include <algorithm>
#include <cstring>
#include <cerrno>
#include <stdexcept>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>

namespace chassis_interface
{

// ============================================================
// FrameParser
// ============================================================
std::vector<std::string> FrameParser::feed(const std::vector<uint8_t>& chunk)
{
  buffer_.insert(buffer_.end(), chunk.begin(), chunk.end());
  std::vector<std::string> messages;
  while (true)
  {
    std::string msg = tryParseOne();
    if (msg.empty()) break;
    messages.push_back(msg);
  }
  return messages;
}

std::string FrameParser::tryParseOne()
{
  if (buffer_.size() < 4) return {};

  std::vector<uint8_t> header = {FRAME_HEADER0, FRAME_HEADER1};
  auto it = std::search(buffer_.begin(), buffer_.end(), header.begin(), header.end());
  if (it != buffer_.begin())
  {
    buffer_.erase(buffer_.begin(), it);
    if (buffer_.size() < 4) return {};
  }

  if (buffer_[0] != FRAME_HEADER0 || buffer_[1] != FRAME_HEADER1)
  {
    buffer_.erase(buffer_.begin());
    return {};
  }

  size_t length = buffer_[2];
  size_t frame_size = length + 4;
  if (buffer_.size() < frame_size) return {};

  std::string payload(buffer_.begin() + 3, buffer_.begin() + 3 + length);
  uint8_t received_cksum = buffer_[3 + length];
  buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);

  if (received_cksum != calcChecksum(payload)) return {};
  return payload;
}

// ============================================================
// 消息解析器
// ============================================================
bool parseNavPose(const std::string& msg, double& x, double& y, double& yaw,
                  double* timestamp)
{
  // nav:time_pose[x,y,radian,time] — 4参数，§37自动上报
  static const std::regex re_time(
      R"(nav:time_pose\[([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+))",
      std::regex::optimize);
  // nav:pose[x,y,radian] — 3参数，§27主动查询
  static const std::regex re_pose(
      R"(nav:pose\[([-\d.]+),([-\d.]+),([-\d.]+))",
      std::regex::optimize);

  std::smatch m;
  try
  {
    if (std::regex_search(msg, m, re_time))
    {
      // m[1]=x, m[2]=y, m[3]=radian, m[4]=timestamp
      x = std::stod(m[1]);
      y = std::stod(m[2]);
      yaw = std::stod(m[3]);
      if (timestamp) *timestamp = std::stod(m[4]);
      return true;
    }
    if (std::regex_search(msg, m, re_pose))
    {
      // m[1]=x, m[2]=y, m[3]=radian
      x = std::stod(m[1]);
      y = std::stod(m[2]);
      yaw = std::stod(m[3]);
      // 3参数格式无时间戳，timestamp保持调用方传入值
      return true;
    }
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM("[LepuDriver] Failed to parse nav_pose: " << msg
                    << " (" << e.what() << ")");
  }
  return false;
}

bool parseBaseVel(const std::string& msg, double& linear, double& angular)
{
  static const std::regex re(R"(base_vel\[([-\d.]+)\s+([-\d.]+))",
                             std::regex::optimize);
  std::smatch m;
  if (!std::regex_search(msg, m, re)) return false;
  try
  {
    linear = std::stod(m[1]);
    angular = std::stod(m[2]);
    ROS_DEBUG("parseBaseVel: linear=%.3f, angular=%.3f", linear, angular);
    return true;
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM("[LepuDriver] Failed to parse base_vel: " << msg
                    << " (" << e.what() << ")");
    return false;
  }
}

bool getNavModeProtocol(const std::string& nav_mode, std::string& command,
                        std::string& expected_response)
{
  if (nav_mode == "navi")
  {
    command = "model:navi";
    expected_response = "model:1";
    return true;
  }
  if (nav_mode == "mapping")
  {
    command = "model:mapping";
    expected_response = "model:2";
    return true;
  }
  if (nav_mode == "remap")
  {
    command = "model:remap";
    expected_response = "model:3";
    return true;
  }
  return false;
}

int parseNavModeResponse(const std::string& msg)
{
  if (msg == "model:1") return 1;
  if (msg == "model:2") return 2;
  if (msg == "model:3") return 3;
  return -1;
}


// ============================================================
// PosixSerial
// ============================================================
static int baudrateToConstant(int baudrate)
{
  switch (baudrate)
  {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 921600:  return B921600;
    default:      return B115200;
  }
}

bool PosixSerial::open(const std::string& port, int baudrate)
{
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) return false;

  tcflush(fd_, TCIOFLUSH);

  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd_, &tty) != 0) { ::close(fd_); fd_ = -1; return false; }

  int speed = baudrateToConstant(baudrate);
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // 8N1, raw mode
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) { ::close(fd_); fd_ = -1; return false; }
  return true;
}

void PosixSerial::close()
{
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

int PosixSerial::read(uint8_t* buf, size_t max_len, int timeout_ms)
{
  if (fd_ < 0) return -1;

  fd_set set;
  FD_ZERO(&set);
  FD_SET(fd_, &set);

  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int ready = select(fd_ + 1, &set, nullptr, nullptr, &tv);
  if (ready == 0) return 0;
  if (ready < 0)
  {
    if (errno == EINTR) return 0;
    return -1;
  }

  ssize_t n = ::read(fd_, buf, max_len);
  if (n == 0) return -1;  // select 后读到 EOF，USB ACM 已断开
  if (n < 0)
  {
    if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) return 0;
    return -1;
  }
  return static_cast<int>(n);
}

int PosixSerial::write(const uint8_t* data, size_t len)
{
  if (fd_ < 0) return -1;
  size_t written = 0;
  while (written < len)
  {
    ssize_t n = ::write(fd_, data + written, len - written);
    if (n < 0)
    {
      if (errno == EINTR) continue;
      return -1;
    }
    if (n == 0) return -1;
    written += static_cast<size_t>(n);
  }
  return static_cast<int>(written);
}

// ============================================================
// LepuSerialLink
// ============================================================
LepuSerialLink::LepuSerialLink(const std::string& port, int baudrate,
                               MessageCallback on_msg,
                               ConnectionCallback on_connection,
                               double reconnect_interval)
  : port_(port)
  , baudrate_(baudrate)
  , on_message_(std::move(on_msg))
  , on_connection_(std::move(on_connection))
  , reconnect_interval_ms_(
      std::max(50, static_cast<int>(reconnect_interval * 1000.0)))
{
}

LepuSerialLink::~LepuSerialLink() { close(); }

bool LepuSerialLink::open()
{
  if (running_.exchange(true)) return true;
  reconnect_requested_ = true;
  read_thread_ = std::thread(&LepuSerialLink::readLoop, this);
  return true;
}

void LepuSerialLink::close()
{
  running_ = false;
  wait_cv_.notify_all();
  if (read_thread_.joinable()) read_thread_.join();
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    connected_ = false;
    serial_.close();
  }
}

bool LepuSerialLink::isOpen() const { return connected_.load(); }

bool LepuSerialLink::sendCommand(const std::string& cmd)
{
  if (!connected_) return false;

  auto frame = buildFrame(cmd);
  bool notify_disconnected = false;
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (!connected_) return false;
    if (serial_.write(frame.data(), frame.size()) != static_cast<int>(frame.size()))
    {
      write_error_count_++;
      notify_disconnected = connected_.exchange(false);
      reconnect_requested_ = true;
    }
  }

  if (notify_disconnected)
  {
    ROS_WARN("[LepuSerialLink] Serial write failed; reconnect requested");
    if (on_connection_) on_connection_(false);
    wait_cv_.notify_all();
    return false;
  }
  return true;
}

void LepuSerialLink::requestReconnect()
{
  reconnect_requested_ = true;
  wait_cv_.notify_all();
}

void LepuSerialLink::markDisconnected(const char* reason)
{
  bool notify_disconnected = false;
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    serial_.close();
    notify_disconnected = connected_.exchange(false);
  }

  if (notify_disconnected)
  {
    ROS_WARN("[LepuSerialLink] %s; reconnect requested", reason);
    if (on_connection_) on_connection_(false);
  }
}

bool LepuSerialLink::waitForRetry()
{
  std::unique_lock<std::mutex> lock(wait_mutex_);
  wait_cv_.wait_for(lock, std::chrono::milliseconds(reconnect_interval_ms_),
                    [this] { return !running_.load(); });
  return running_.load();
}

void LepuSerialLink::readLoop()
{
  while (running_)
  {
    if (reconnect_requested_.exchange(false))
    {
      markDisconnected("Reconnect explicitly requested");
    }

    if (!connected_)
    {
      bool opened = false;
      {
        std::lock_guard<std::mutex> lock(write_mutex_);
        if (running_ && serial_.open(port_, baudrate_))
        {
          connected_ = true;
          opened = true;
        }
      }

      if (opened)
      {
        parser_ = FrameParser();
        if (ever_connected_)
          reconnect_count_++;
        else
          ever_connected_ = true;
        ROS_INFO("[LepuSerialLink] Connected to %s", port_.c_str());
        if (on_connection_) on_connection_(true);
        continue;
      }

      ROS_WARN_THROTTLE(5.0, "[LepuSerialLink] Waiting for serial port %s",
                        port_.c_str());
      if (!waitForRetry()) break;
      continue;
    }

    uint8_t buf[256];
    int n = serial_.read(buf, sizeof(buf), 50);
    if (n < 0)
    {
      read_error_count_++;
      markDisconnected("Serial read returned EOF or fatal error");
      continue;
    }
    if (n == 0)
    {
      continue;
    }

    std::vector<uint8_t> chunk(buf, buf + n);
    for (const auto& msg : parser_.feed(chunk))
    {
      if (on_message_) on_message_(msg);
    }
  }
}

}  // namespace chassis_interface
