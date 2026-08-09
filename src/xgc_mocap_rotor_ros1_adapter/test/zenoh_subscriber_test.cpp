#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <zenoh.h>

#include "xgc_mocap_rotor_ros1_adapter/zenoh_subscriber.hpp"

namespace xgc_mocap_rotor_ros1_adapter {
namespace {

std::uint16_t unusedLoopbackPort() {
  const int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd < 0)
    throw std::runtime_error("cannot create loopback port probe socket");
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = 0;
  if (bind(socket_fd, reinterpret_cast<sockaddr *>(&address),
           sizeof(address)) != 0) {
    close(socket_fd);
    throw std::runtime_error("cannot bind loopback port probe socket");
  }
  socklen_t address_size = sizeof(address);
  if (getsockname(socket_fd, reinterpret_cast<sockaddr *>(&address),
                  &address_size) != 0) {
    close(socket_fd);
    throw std::runtime_error("cannot read loopback port probe socket");
  }
  close(socket_fd);
  return ntohs(address.sin_port);
}

TEST(MocapRotorZenohSubscriber, ReceivesRobotKeyedUplinkOverOnePeerSession) {
  std::mutex mutex;
  std::condition_variable received_condition;
  std::string received_key;
  std::string received_payload;
  ZenohSubscriber subscriber(
      [&](std::string key, std::string payload) {
        std::lock_guard<std::mutex> lock(mutex);
        received_key = std::move(key);
        received_payload = std::move(payload);
        received_condition.notify_all();
      });

  const std::string endpoint =
      "tcp/127.0.0.1:" + std::to_string(unusedLoopbackPort());
  std::string error;
  ASSERT_TRUE(subscriber.Start(endpoint, &error)) << error;
  ASSERT_TRUE(subscriber.running());

  z_owned_config_t config;
  z_internal_null(&config);
  ASSERT_GE(z_config_default(&config), 0);
  const std::string connect = "['" + endpoint + "']";
  ASSERT_GE(zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_MODE_KEY,
                                   "'peer'"),
            0);
  ASSERT_GE(zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_CONNECT_KEY,
                                   connect.c_str()),
            0);
  ASSERT_GE(zc_config_insert_json5(z_loan_mut(config),
                                   Z_CONFIG_MULTICAST_SCOUTING_KEY, "false"),
            0);
  ASSERT_GE(zc_config_insert_json5(z_loan_mut(config),
                                   "scouting/gossip/enabled", "false"),
            0);
  z_owned_session_t publisher;
  z_internal_null(&publisher);
  ASSERT_GE(z_open(&publisher, z_move(config), nullptr), 0);

  const std::string key = "xgc2/mocap-rotor-01/up/local_pose";
  const std::string payload =
      R"({"v":1,"sequence":1,"t_ms":1000,"frame_id":"world"})";
  z_view_keyexpr_t key_expression;
  ASSERT_GE(z_view_keyexpr_from_str(&key_expression, key.c_str()), 0);

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  std::unique_lock<std::mutex> lock(mutex);
  while (received_key.empty() && std::chrono::steady_clock::now() < deadline) {
    lock.unlock();
    z_owned_bytes_t bytes;
    z_bytes_from_static_str(&bytes, payload.c_str());
    ASSERT_GE(z_put(z_loan(publisher), z_loan(key_expression), z_move(bytes),
                    nullptr),
              0);
    lock.lock();
    received_condition.wait_for(lock, std::chrono::milliseconds(100));
  }
  EXPECT_EQ(received_key, key);
  EXPECT_EQ(received_payload, payload);
  lock.unlock();

  z_drop(z_move(publisher));
  subscriber.Stop();
  EXPECT_FALSE(subscriber.running());
}

} // namespace
} // namespace xgc_mocap_rotor_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
