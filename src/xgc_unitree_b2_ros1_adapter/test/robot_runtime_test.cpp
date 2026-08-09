#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <sys/socket.h>
#include <unistd.h>

#include "xgc_unitree_b2_ros1_adapter/robot_runtime.hpp"
#include "xgc_unitree_b2_ros1_adapter/wire_tcp_server.hpp"

namespace xgc_unitree_b2_ros1_adapter {

TEST(WireContract, AcceptsOnlyReadOnlyKeysForExactRobot) {
  std::string channel;
  EXPECT_TRUE(validWireKey("b2-01", "xgc2/b2-01/up/odom", &channel));
  EXPECT_EQ("odom", channel);
  EXPECT_TRUE(validWireKey("b2-01", "xgc2/b2-01/up/arm_slave_status", &channel));
  EXPECT_FALSE(validWireKey("b2-01", "xgc2/b2-01/down/cmd", &channel));
  EXPECT_FALSE(validWireKey("b2-01", "xgc2/b2-02/up/odom", &channel));
  EXPECT_FALSE(validWireKey("b2-01", "xgc2/b2-01/up/unknown", &channel));
}

TEST(WireContract, ValidatesListenPortAndHost) {
  std::string error;
  std::uint16_t port = 0;
  EXPECT_TRUE(validWireHost("0.0.0.0", &error));
  EXPECT_FALSE(validWireHost("core", &error));
  EXPECT_TRUE(parseWirePort("7448", &port, &error));
  EXPECT_EQ(7448u, port);
  EXPECT_FALSE(parseWirePort("0", &port, &error));
  EXPECT_FALSE(parseWirePort("65536", &port, &error));
}

TEST(WireContract, ReceivesTheFrozenBigEndianFrame) {
  const int reservation = ::socket(AF_INET, SOCK_STREAM, 0);
  ASSERT_GE(reservation, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = 0;
  ASSERT_EQ(0, ::bind(reservation, reinterpret_cast<sockaddr *>(&address), sizeof(address)));
  socklen_t address_size = sizeof(address);
  ASSERT_EQ(0, ::getsockname(reservation, reinterpret_cast<sockaddr *>(&address), &address_size));
  const std::uint16_t port = ntohs(address.sin_port);
  ::close(reservation);

  std::mutex mutex;
  std::condition_variable received;
  std::string received_key;
  std::string received_payload;
  WireTcpServer server("127.0.0.1", port,
      [&](const std::string &key, const std::string &payload) {
        std::lock_guard<std::mutex> lock(mutex);
        received_key = key;
        received_payload = payload;
        received.notify_all();
      });
  std::string error;
  ASSERT_TRUE(server.Start(&error)) << error;

  const int client = ::socket(AF_INET, SOCK_STREAM, 0);
  ASSERT_GE(client, 0);
  ASSERT_EQ(0, ::connect(client, reinterpret_cast<sockaddr *>(&address), sizeof(address)));
  const std::string key = "xgc2/b2-01/up/odom";
  const std::string payload = "{\"v\":1}";
  const std::uint32_t key_size = htonl(static_cast<std::uint32_t>(key.size()));
  const std::uint32_t payload_size = htonl(static_cast<std::uint32_t>(payload.size()));
  ASSERT_EQ(static_cast<ssize_t>(sizeof(key_size)), ::send(client, &key_size, sizeof(key_size), 0));
  ASSERT_EQ(static_cast<ssize_t>(key.size()), ::send(client, key.data(), key.size(), 0));
  ASSERT_EQ(static_cast<ssize_t>(sizeof(payload_size)), ::send(client, &payload_size, sizeof(payload_size), 0));
  ASSERT_EQ(static_cast<ssize_t>(payload.size()), ::send(client, payload.data(), payload.size(), 0));
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(received.wait_for(lock, std::chrono::seconds(2), [&] {
      return !received_key.empty();
    }));
  }
  EXPECT_EQ(key, received_key);
  EXPECT_EQ(payload, received_payload);
  ::close(client);
  server.Stop();
}

TEST(Freshness, UsesGroundReceiveMonotonicTime) {
  const ros::WallTime now(20, 0);
  EXPECT_TRUE(sourceIsFresh(ros::WallTime(19, 500000000), now, 1.0));
  EXPECT_FALSE(sourceIsFresh(ros::WallTime(18, 0), now, 1.0));
  EXPECT_FALSE(sourceIsFresh(ros::WallTime(), now, 1.0));
}

}  // namespace xgc_unitree_b2_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
