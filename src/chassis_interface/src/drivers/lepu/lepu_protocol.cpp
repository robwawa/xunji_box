#include <chassis_interface/drivers/lepu/lepu_protocol.h>
#include <ros/console.h>
#include <regex>
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

  if (select(fd_ + 1, &set, nullptr, nullptr, &tv) <= 0) return 0;
  ssize_t n = ::read(fd_, buf, max_len);
  if (n < 0 && errno == EINTR) return 0;
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
    written += static_cast<size_t>(n);
  }
  return static_cast<int>(written);
}

// ============================================================
// LepuSerialLink
// ============================================================
LepuSerialLink::LepuSerialLink(const std::string& port, int baudrate,
                               MessageCallback on_msg)
  : port_(port), baudrate_(baudrate), on_message_(std::move(on_msg))
{
}

LepuSerialLink::~LepuSerialLink() { close(); }

bool LepuSerialLink::open()
{
  if (!serial_.open(port_, baudrate_)) return false;
  running_ = true;
  read_thread_ = std::thread(&LepuSerialLink::readLoop, this);
  return true;
}

void LepuSerialLink::close()
{
  running_ = false;
  if (read_thread_.joinable()) read_thread_.join();
  serial_.close();
}

bool LepuSerialLink::isOpen() const { return serial_.isOpen(); }

void LepuSerialLink::sendCommand(const std::string& cmd)
{
  if (!serial_.isOpen()) return;
  auto frame = buildFrame(cmd);
  std::lock_guard<std::mutex> lock(write_mutex_);
  serial_.write(frame.data(), frame.size());
}

void LepuSerialLink::readLoop()
{
  while (running_)
  {
    if (!serial_.isOpen())
    {
      // 自动重连
      if (serial_.open(port_, baudrate_))
      {
        // 重连成功，清空解析器缓冲区
        parser_ = FrameParser();
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      continue;
    }

    uint8_t buf[256];
    int n = serial_.read(buf, sizeof(buf), 50);
    if (n <= 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
