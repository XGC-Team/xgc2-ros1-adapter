#include "xgc2_ros1_robot_adapter/robot_server_entry.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <sys/socket.h>
#include <vector>

using namespace xgc2_ros1_robot_adapter;
namespace {
RobotServerArguments parse(std::initializer_list<std::string> flags) {
  std::vector<std::string> values{"robot-server"};
  values.insert(values.end(), flags.begin(), flags.end());
  std::vector<char *> argv;
  for (auto &value : values) argv.push_back(&value[0]);
  return ParseRobotServerArguments(static_cast<int>(argv.size()), argv.data(), "native");
}
struct PrivateDirectory {
  std::string path;
  PrivateDirectory() { char pattern[] = "/tmp/robot-entry-XXXXXX"; path = mkdtemp(pattern); }
  ~PrivateDirectory() { rmdir(path.c_str()); }
};
int bindSocket(const std::string &path) {
  int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  sockaddr_un address{}; address.sun_family = AF_UNIX;
  std::strcpy(address.sun_path, path.c_str());
  if (bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address))) { close(fd); return -1; }
  return fd;
}
TEST(RobotServerEntry, NativeAndBootstrapModesPreserveTheirInputs) {
  auto args = parse({"--socket-path", "/run/xgc2/robot/server.sock", "--provider", "native",
                     "--ros-master-uri", "http://127.0.0.1:11311", "--ros-ip", "127.0.0.1"});
  EXPECT_EQ(args.provider, "native"); EXPECT_EQ(args.ros_ip, "127.0.0.1");
  EXPECT_EQ(args.ros_master_uri, "http://127.0.0.1:11311"); EXPECT_FALSE(args.check);
  EXPECT_EQ(parse({"--adapter-bootstrap-file", "/tmp/private.pb"}).bootstrap_file, "/tmp/private.pb");
  EXPECT_EQ(parse({"--socket-path", "/tmp/native.sock", "--provider", "native"}).ros_ip, "");
  EXPECT_EQ(parse({"--socket-path", "/tmp/native.sock", "--provider", "native", "--ros-ip", ""}).ros_ip, "");
}
TEST(RobotServerEntry, CheckIsFiniteAndHasNoProviderOrRosSettings) {
  EXPECT_EQ(parse({"--check", "--socket-path", "/tmp/health.sock"}).timeout_ms, 2000);
  EXPECT_EQ(parse({"--check", "--socket-path", "/tmp/health.sock", "--timeout-ms", "60000"}).timeout_ms, 60000);
  for (const auto &value : {"0", "60001", "-1", "1.0", "1junk", "99999999999999999999999"})
    EXPECT_THROW(parse({"--check", "--socket-path", "/tmp/health.sock", "--timeout-ms", value}), std::runtime_error);
  EXPECT_THROW(parse({"--check", "--socket-path", "/tmp/health.sock", "--provider", "native"}), std::runtime_error);
  EXPECT_THROW(parse({"--check", "--socket-path", "/tmp/health.sock", "--ros-ip", "127.0.0.1"}), std::runtime_error);
  EXPECT_THROW(parse({"--socket-path", "/tmp/health.sock", "--provider", "native", "--timeout-ms", "1"}), std::runtime_error);
}
TEST(RobotServerEntry, RejectsMissingRepeatedMixedAndWrongProvider) {
  EXPECT_THROW(parse({}), std::runtime_error);
  for (const auto &flag : {"--socket-path", "--provider", "--ros-ip", "--ros-master-uri", "--adapter-bootstrap-file", "--timeout-ms"}) {
    EXPECT_THROW(parse({flag}), std::runtime_error);
    EXPECT_THROW(parse({flag, "--check"}), std::runtime_error);
    if (std::string(flag) != "--ros-ip") {
      EXPECT_THROW(parse({flag, ""}), std::runtime_error);
    }
    EXPECT_THROW(parse({flag, "value", flag, "value"}), std::runtime_error);
  }
  EXPECT_THROW(parse({"--check", "--check", "--socket-path", "/tmp/a.sock"}), std::runtime_error);
  EXPECT_THROW(parse({"--adapter-bootstrap-file", "/tmp/a.pb", "--socket-path", "/tmp/a.sock"}), std::runtime_error);
  EXPECT_THROW(parse({"--adapter-bootstrap-file", "/tmp/a.pb", "--check"}), std::runtime_error);
  EXPECT_THROW(parse({"--socket-path", "/tmp/a.sock", "--provider", "foreign"}), std::runtime_error);
  EXPECT_THROW(parse({"--unknown", "value"}), std::runtime_error);
}
TEST(RobotServerEntry, RejectsAmbiguousOrOverlongSocketPaths) {
  for (const auto &path : {"relative.sock", "/", "/tmp//a.sock", "/tmp/./a.sock", "/tmp/../a.sock", "/tmp/a/"})
    EXPECT_THROW(ValidateRobotSocketPath(path), std::runtime_error);
  EXPECT_THROW(ValidateRobotSocketPath("/" + std::string(108, 'x')), std::runtime_error);
}
TEST(RobotServerEntry, CreatesOnlyMissingPrivateParents) {
  PrivateDirectory root;
  const auto directory = root.path + "/private";
  EnsurePrivateRobotSocketParent(directory + "/server.sock");
  EnsurePrivateRobotSocketParent(directory + "/server.sock");
  struct stat state{}; ASSERT_EQ(stat(directory.c_str(), &state), 0);
  EXPECT_EQ(state.st_mode & 0777, 0700u);
  EXPECT_EQ(access((directory + "/server.sock").c_str(), F_OK), -1);
  rmdir(directory.c_str());
}
TEST(RobotServerEntry, PreservesExistingPublicDirectoriesSymlinksAndFiles) {
  PrivateDirectory root;
  const auto public_directory = root.path + "/public";
  ASSERT_EQ(mkdir(public_directory.c_str(), 0755), 0);
  EXPECT_THROW(EnsurePrivateRobotSocketParent(public_directory + "/server.sock"), std::runtime_error);
  struct stat state{}; ASSERT_EQ(stat(public_directory.c_str(), &state), 0); EXPECT_EQ(state.st_mode & 0777, 0755u);
  const auto link = root.path + "/link"; ASSERT_EQ(symlink(public_directory.c_str(), link.c_str()), 0);
  EXPECT_THROW(EnsurePrivateRobotSocketParent(link + "/server.sock"), std::runtime_error);
  ASSERT_EQ(lstat(link.c_str(), &state), 0); EXPECT_TRUE(S_ISLNK(state.st_mode));
  const auto file = root.path + "/file"; std::ofstream(file) << "foreign";
  EXPECT_THROW(EnsurePrivateRobotSocketParent(file + "/server.sock"), std::runtime_error);
  unlink(file.c_str()); unlink(link.c_str()); rmdir(public_directory.c_str());
}
TEST(RobotServerEntry, RefusesPreexistingForeignSocketAndFile) {
  PrivateDirectory root; const auto path = root.path + "/server.sock";
  const int socket = bindSocket(path); ASSERT_GE(socket, 0);
  struct stat before{}, after{}; ASSERT_EQ(lstat(path.c_str(), &before), 0);
  EXPECT_THROW(RobotSocketOwner owner(path), std::runtime_error);
  ASSERT_EQ(lstat(path.c_str(), &after), 0); EXPECT_EQ(before.st_ino, after.st_ino);
  close(socket); unlink(path.c_str());
  std::ofstream(path) << "foreign";
  EXPECT_THROW(RobotSocketOwner owner(path), std::runtime_error);
  EXPECT_EQ(access(path.c_str(), F_OK), 0); unlink(path.c_str());
}
TEST(RobotServerEntry, CleanupRemovesOnlyItsExactSocketInode) {
  PrivateDirectory root; const auto path = root.path + "/server.sock";
  int original = -1, replacement = -1;
  {
    RobotSocketOwner owner(path); original = bindSocket(path); ASSERT_GE(original, 0); owner.RecordBoundSocket();
    struct stat state{}; ASSERT_EQ(lstat(path.c_str(), &state), 0); EXPECT_EQ(state.st_mode & 0777, 0600u);
  }
  EXPECT_EQ(access(path.c_str(), F_OK), -1); close(original);
  {
    RobotSocketOwner owner(path); original = bindSocket(path); ASSERT_GE(original, 0); owner.RecordBoundSocket();
    ASSERT_EQ(rename(path.c_str(), (path + ".owned").c_str()), 0);
    replacement = bindSocket(path); ASSERT_GE(replacement, 0);
  }
  EXPECT_EQ(access(path.c_str(), F_OK), 0);
  close(original); close(replacement); unlink(path.c_str()); unlink((path + ".owned").c_str());
}
} // namespace
int main(int argc, char **argv) { testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
