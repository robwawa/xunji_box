#include <chassis_interface/drivers/lepu/lepu_protocol.h>

#include <gtest/gtest.h>
#include <ros/time.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <string>
#include <thread>

#include <pty.h>
#include <unistd.h>

namespace chassis_interface
{
namespace
{

bool waitUntil(const std::function<bool()>& predicate, int timeout_ms = 3000)
{
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (predicate()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return predicate();
}

int createPty(std::string& slave_path)
{
  int master_fd = -1;
  int slave_fd = -1;
  char name[128] = {};
  if (openpty(&master_fd, &slave_fd, name, nullptr, nullptr) != 0) return -1;
  slave_path = name;
  ::close(slave_fd);
  return master_fd;
}

TEST(LepuProtocolTest, MapsAndParsesNavModesExactly)
{
  std::string command;
  std::string response;

  ASSERT_TRUE(getNavModeProtocol("navi", command, response));
  EXPECT_EQ("model:navi", command);
  EXPECT_EQ("model:1", response);

  ASSERT_TRUE(getNavModeProtocol("mapping", command, response));
  EXPECT_EQ("model:mapping", command);
  EXPECT_EQ("model:2", response);
  EXPECT_NE(response, "model:1");

  ASSERT_TRUE(getNavModeProtocol("remap", command, response));
  EXPECT_EQ("model:remap", command);
  EXPECT_EQ("model:3", response);

  EXPECT_FALSE(getNavModeProtocol("invalid", command, response));
  EXPECT_EQ(1, parseNavModeResponse("model:1"));
  EXPECT_EQ(2, parseNavModeResponse("model:2"));
  EXPECT_EQ(3, parseNavModeResponse("model:3"));
  EXPECT_EQ(-1, parseNavModeResponse("model:2\r"));
  EXPECT_EQ(-1, parseNavModeResponse("nav:pose[1,2,0]"));
}

TEST(PosixSerialTest, DistinguishesTimeoutFromDeviceHangup)
{
  ros::Time::init();
  std::string slave_path;
  int master_fd = createPty(slave_path);
  ASSERT_GE(master_fd, 0);

  PosixSerial serial;
  ASSERT_TRUE(serial.open(slave_path, 115200));

  uint8_t byte = 0;
  EXPECT_EQ(0, serial.read(&byte, 1, 20));

  const uint8_t expected = 0x42;
  ASSERT_EQ(1, ::write(master_fd, &expected, 1));
  ASSERT_TRUE(waitUntil([&] { return serial.read(&byte, 1, 20) == 1; }));
  EXPECT_EQ(expected, byte);

  ::close(master_fd);
  EXPECT_TRUE(waitUntil([&] { return serial.read(&byte, 1, 20) < 0; }));
}

TEST(LepuSerialLinkTest, ReopensStableAliasAfterDeviceRenumeration)
{
  ros::Time::init();
  char temp_dir_template[] = "/tmp/lepu_serial_test_XXXXXX";
  char* temp_dir = ::mkdtemp(temp_dir_template);
  ASSERT_NE(nullptr, temp_dir);
  const std::string stable_path = std::string(temp_dir) + "/lepu_chassis";

  std::string slave_a;
  int master_a = createPty(slave_a);
  ASSERT_GE(master_a, 0);
  ASSERT_EQ(0, ::symlink(slave_a.c_str(), stable_path.c_str()));

  std::atomic<int> connected_events{0};
  std::atomic<int> disconnected_events{0};
  std::atomic<int> message_count{0};
  LepuSerialLink link(
      stable_path, 115200,
      [&](const std::string& msg) {
        if (msg == "nav:pose[1,2,0.1]") message_count++;
      },
      [&](bool connected) {
        if (connected)
          connected_events++;
        else
          disconnected_events++;
      },
      0.05);

  ASSERT_TRUE(link.open());
  ASSERT_TRUE(waitUntil([&] { return connected_events.load() == 1; }));

  auto frame = buildFrame("nav:pose[1,2,0.1]");
  ASSERT_EQ(static_cast<ssize_t>(frame.size()),
            ::write(master_a, frame.data(), frame.size()));
  ASSERT_TRUE(waitUntil([&] { return message_count.load() == 1; }));

  ::close(master_a);
  ASSERT_TRUE(waitUntil([&] { return disconnected_events.load() == 1; }));

  std::string slave_b;
  int master_b = createPty(slave_b);
  ASSERT_GE(master_b, 0);
  ASSERT_EQ(0, ::unlink(stable_path.c_str()));
  ASSERT_EQ(0, ::symlink(slave_b.c_str(), stable_path.c_str()));

  ASSERT_TRUE(waitUntil([&] {
    return connected_events.load() == 2 && link.reconnectCount() == 1;
  }));
  ASSERT_EQ(static_cast<ssize_t>(frame.size()),
            ::write(master_b, frame.data(), frame.size()));
  ASSERT_TRUE(waitUntil([&] { return message_count.load() == 2; }));

  link.close();
  ::close(master_b);
  ::unlink(stable_path.c_str());
  ::rmdir(temp_dir);
}

}  // namespace
}  // namespace chassis_interface

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
