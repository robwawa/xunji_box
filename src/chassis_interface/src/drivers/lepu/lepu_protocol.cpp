#include <chassis_interface/drivers/lepu/lepu_protocol.h>
#include <regex>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <cerrno>

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
bool parseNavPose(const std::string& msg, double& x, double& y, double& yaw)
{
  std::regex re(R"(nav:time_pose\[([-\d.]+),([-\d.]+),([-\d.]+))");
  std::smatch m;
  if (std::regex_search(msg, m, re))
  {
    x = std::stod(m[1]); y = std::stod(m[2]); yaw = std::stod(m[3]);
    return true;
  }
  re = R"(nav:pose\[([-\d.]+),([-\d.]+),([-\d.]+))";
  if (std::regex_search(msg, m, re))
  {
    x = std::stod(m[1]); y = std::stod(m[2]); yaw = std::stod(m[3]);
    return true;
  }
  return false;
}

bool parseBaseVel(const std::string& msg, double& linear, double& angular)
{
  std::regex re(R"(base_vel\[([-\d.]+)\s+([-\d.]+))");
  std::smatch m;
  if (!std::regex_search(msg, m, re)) return false;
  linear = std::stod(m[1]); angular = std::stod(m[2]);
  return true;
}

static bool parseArrayField6_7(const std::string& msg, const std::string& tag,
                                int32_t& left, int32_t& right)
{
  std::regex re(tag + R"(\{([^}]+))");
  std::smatch m;
  if (!std::regex_search(msg, m, re)) return false;

  std::istringstream iss(m[1].str());
  std::vector<double> values;
  double val;
  while (iss >> val) values.push_back(val);
  if (values.size() < 7) return false;
  left = static_cast<int32_t>(values[5]);
  right = static_cast<int32_t>(values[6]);
  return true;
}

bool parseWheelEncoders(const std::string& msg, int32_t& left, int32_t& right)
{
  if (parseArrayField6_7(msg, "core_data", left, right)) return true;
  return parseArrayField6_7(msg, "wheel_status", left, right);
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
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
