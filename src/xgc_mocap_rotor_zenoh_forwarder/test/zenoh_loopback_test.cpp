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

#include "xgc_mocap_rotor_zenoh_forwarder/zenoh_publisher.hpp"

namespace xgc_mocap_rotor_zenoh_forwarder {
namespace {

struct ReceivedSample {
  std::mutex mutex;
  std::condition_variable condition;
  std::string key;
  std::string payload;
};

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

void receive(z_loaned_sample_t *sample, void *context) {
  auto *received = static_cast<ReceivedSample *>(context);
  if (received == nullptr || sample == nullptr ||
      z_sample_kind(sample) != Z_SAMPLE_KIND_PUT) {
    return;
  }
  z_view_string_t key;
  z_keyexpr_as_view_string(z_sample_keyexpr(sample), &key);
  z_owned_string_t payload;
  z_internal_null(&payload);
  if (z_bytes_to_string(z_sample_payload(sample), &payload) < 0)
    return;
  const auto *key_value = z_loan(key);
  const auto *payload_value = z_loan(payload);
  if (key_value != nullptr && payload_value != nullptr) {
    std::lock_guard<std::mutex> lock(received->mutex);
    received->key.assign(z_string_data(key_value), z_string_len(key_value));
    received->payload.assign(z_string_data(payload_value),
                             z_string_len(payload_value));
    received->condition.notify_all();
  }
  z_drop(z_move(payload));
}

TEST(MocapRotorZenohForwarder, SendsOneRobotKeyedUplinkToGroundPeer) {
  const std::string endpoint =
      "tcp/127.0.0.1:" + std::to_string(unusedLoopbackPort());
  z_owned_config_t config;
  z_internal_null(&config);
  ASSERT_GE(z_config_default(&config), 0);
  const std::string listen = "['" + endpoint + "']";
  ASSERT_GE(zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_MODE_KEY,
                                   "'peer'"),
            0);
  ASSERT_GE(zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_LISTEN_KEY,
                                   listen.c_str()),
            0);
  ASSERT_GE(zc_config_insert_json5(z_loan_mut(config),
                                   Z_CONFIG_MULTICAST_SCOUTING_KEY, "false"),
            0);
  ASSERT_GE(zc_config_insert_json5(z_loan_mut(config),
                                   "scouting/gossip/enabled", "false"),
            0);
  z_owned_session_t ground;
  z_internal_null(&ground);
  ASSERT_GE(z_open(&ground, z_move(config), nullptr), 0);

  ReceivedSample received;
  z_view_keyexpr_t selector;
  ASSERT_GE(z_view_keyexpr_from_str(&selector, "xgc2/*/up/**"), 0);
  z_owned_closure_sample_t callback;
  z_closure(&callback, receive, nullptr, &received);
  z_owned_subscriber_t subscriber;
  z_internal_null(&subscriber);
  ASSERT_GE(z_declare_subscriber(z_loan(ground), &subscriber, z_loan(selector),
                                 z_move(callback), nullptr),
            0);

  ZenohPublisher publisher;
  std::string error;
  ASSERT_TRUE(publisher.Start("mocap-rotor-01", endpoint, &error)) << error;
  const std::string payload = R"({"v":1,"sequence":1,"t_ms":1000})";
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  std::unique_lock<std::mutex> lock(received.mutex);
  while (received.key.empty() && std::chrono::steady_clock::now() < deadline) {
    lock.unlock();
    ASSERT_TRUE(publisher.Put(UplinkChannel::kLocalPose, payload, &error))
        << error;
    lock.lock();
    received.condition.wait_for(lock, std::chrono::milliseconds(100));
  }
  EXPECT_EQ(received.key, "xgc2/mocap-rotor-01/up/local_pose");
  EXPECT_EQ(received.payload, payload);
  lock.unlock();

  publisher.Stop();
  z_drop(z_move(subscriber));
  z_drop(z_move(ground));
}

} // namespace
} // namespace xgc_mocap_rotor_zenoh_forwarder

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
