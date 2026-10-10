#include "xgc2_ros1_robot_adapter/robot_server.hpp"
#include <gtest/gtest.h>
#include <vector>

namespace xgc2_ros1_robot_adapter {
namespace {

const std::vector<std::string> kProviders{"type-a", "type-b"};

std::vector<std::string> complete() {
  return {"--socket-path", "/run/xgc2/sockets/type-a.sock",
          "--run-id",      "0b1f6a6e-5f31-4a4f-9d31-0e7a8f0c1d11",
          "--node-name",   "xgc_type_a_0b1f6a6e",
          "--provider",    "type-a",
          "--target-id",   "local",
          "--ros-master-uri", "http://127.0.0.1:11311",
          "--ros-ip",      "127.0.0.1"};
}

ServerOptions parseAll(const std::vector<std::string> &flags) {
  std::vector<std::string> values{"robot-server"};
  values.insert(values.end(), flags.begin(), flags.end());
  std::vector<char *> argv;
  for (auto &value : values)
    argv.push_back(&value[0]);
  return ParseServerOptions(static_cast<int>(argv.size()), argv.data(), kProviders);
}

TEST(ServerOptions, ParseEveryFlagOfTheRun) {
  const auto options = parseAll(complete());
  EXPECT_EQ("/run/xgc2/sockets/type-a.sock", options.socket_path);
  EXPECT_EQ("0b1f6a6e-5f31-4a4f-9d31-0e7a8f0c1d11", options.run_id);
  EXPECT_EQ("xgc_type_a_0b1f6a6e", options.node_name);
  EXPECT_EQ("type-a", options.provider);
  EXPECT_EQ("local", options.target_id);
  EXPECT_EQ("http://127.0.0.1:11311", options.ros_master_uri);
  EXPECT_EQ("127.0.0.1", options.ros_ip);
}

TEST(ServerOptions, TheRosAddressMayBeEmptyButNothingElse) {
  auto flags = complete();
  flags.back() = "";
  EXPECT_EQ("", parseAll(flags).ros_ip);
  flags = complete();
  flags.erase(flags.end() - 2, flags.end());
  EXPECT_EQ("", parseAll(flags).ros_ip);
  for (std::size_t value = 1; value < complete().size(); value += 2) {
    if (complete()[value - 1] == "--ros-ip")
      continue;
    flags = complete();
    flags[value] = "";
    EXPECT_THROW(parseAll(flags), std::invalid_argument) << complete()[value - 1];
  }
}

TEST(ServerOptions, EveryRequiredFlagMustBePresentAndNotRepeated) {
  for (std::size_t flag = 0; flag < complete().size(); flag += 2) {
    auto flags = complete();
    const std::string name = flags[flag];
    flags.erase(flags.begin() + flag, flags.begin() + flag + 2);
    if (name == "--ros-ip") {
      EXPECT_NO_THROW(parseAll(flags));
    } else {
      EXPECT_THROW(parseAll(flags), std::invalid_argument) << name;
    }
    flags = complete();
    flags.push_back(name);
    flags.push_back("value");
    EXPECT_THROW(parseAll(flags), std::invalid_argument) << name;
  }
}

TEST(ServerOptions, RejectsRetiredAndUnknownFlags) {
  for (const char *retired : {"--check", "--timeout-ms", "--adapter-bootstrap-file", "--unknown"}) {
    auto flags = complete();
    flags.push_back(retired);
    flags.push_back("value");
    EXPECT_THROW(parseAll(flags), std::invalid_argument) << retired;
  }
  auto flags = complete();
  flags.push_back("--check");
  EXPECT_THROW(parseAll(flags), std::invalid_argument);
}

TEST(ServerOptions, ServesOnlyTheProvidersOfTheExecutable) {
  auto flags = complete();
  flags[7] = "type-b";
  EXPECT_EQ("type-b", parseAll(flags).provider);
  flags[7] = "foreign";
  EXPECT_THROW(parseAll(flags), std::invalid_argument);
}

TEST(ServerOptions, ValidatesSocketRunAndMasterSyntax) {
  auto bad = [](std::size_t index, const std::string &value) {
    auto flags = complete();
    flags[index] = value;
    EXPECT_THROW(parseAll(flags), std::invalid_argument) << value;
  };
  for (const char *path : {"relative.sock", "/", "/tmp//a.sock", "/tmp/./a.sock",
                           "/tmp/../a.sock", "/tmp/a/"})
    bad(1, path);
  bad(1, "/" + std::string(108, 'x'));
  bad(3, "run id");
  bad(3, std::string(129, 'r'));
  bad(9, "bad target");
  for (const char *uri : {"127.0.0.1:11311", "https://127.0.0.1:11311",
                          "http://127.0.0.1", "http://:11311",
                          "http://127.0.0.1:0", "http://127.0.0.1:70000",
                          "http://127.0.0.1:11311/xmlrpc"})
    bad(11, uri);
  auto flags = complete();
  flags[11] = "http://localhost:11311/";
  EXPECT_NO_THROW(parseAll(flags));
}

TEST(NodeName, KeepsOnlyCharactersRosAllows) {
  EXPECT_EQ("xgc_scout_0b1f6a6e_5f31", RosNodeName("xgc_scout_0b1f6a6e-5f31"));
  EXPECT_EQ("n1abc", RosNodeName("1abc"));
  EXPECT_EQ("n_run", RosNodeName("_run"));
  EXPECT_EQ("n", RosNodeName(""));
  EXPECT_EQ("a_b_c", RosNodeName("a.b/c"));
}

TEST(ReferenceId, FollowsTheXrpcIdentifierGrammar) {
  EXPECT_TRUE(ValidReferenceId("run-1.a_b:c"));
  EXPECT_FALSE(ValidReferenceId(""));
  EXPECT_FALSE(ValidReferenceId("has space"));
  EXPECT_FALSE(ValidReferenceId(std::string(129, 'x')));
  EXPECT_TRUE(ValidReferenceId(std::string(128, 'x')));
}

} // namespace
} // namespace xgc2_ros1_robot_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
