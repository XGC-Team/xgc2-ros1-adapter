#pragma once

#include <cerrno>
#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <set>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace xgc2_ros1_robot_adapter {
struct RobotServerArguments {
  std::string bootstrap_file, socket_path, provider, ros_master_uri, ros_ip;
  bool check = false;
  int timeout_ms = 2000;
};

inline void ValidateRobotSocketPath(const std::string &path) {
  if (path.empty() || path.front() != '/' || path.find('\0') != std::string::npos ||
      path.size() >= sizeof(sockaddr_un::sun_path))
    throw std::runtime_error("socket path must be a bounded absolute Unix path");
  std::size_t start = 1;
  for (;;) {
    const auto end = path.find('/', start);
    const auto component = path.substr(start, end == std::string::npos ? end : end - start);
    if (component.empty() || component == "." || component == "..")
      throw std::runtime_error("socket path contains an invalid component");
    if (end == std::string::npos) break;
    start = end + 1;
  }
}

inline RobotServerArguments ParseRobotServerArguments(int argc, char **argv,
                                                     const std::string &native_provider) {
  RobotServerArguments result;
  std::set<std::string> seen;
  for (int index = 1; index < argc; ++index) {
    const std::string flag(argv[index]);
    if (flag != "--adapter-bootstrap-file" && flag != "--socket-path" &&
        flag != "--provider" && flag != "--ros-master-uri" && flag != "--ros-ip" &&
        flag != "--check" && flag != "--timeout-ms")
      throw std::runtime_error("unknown argument: " + flag);
    if (!seen.insert(flag).second) throw std::runtime_error("argument occurs more than once: " + flag);
    if (flag == "--check") { result.check = true; continue; }
    if (index + 1 >= argc || std::string(argv[index + 1]).compare(0, 2, "--") == 0)
      throw std::runtime_error("missing value for " + flag);
    const std::string value(argv[++index]);
    if ((value.empty() && flag != "--ros-ip") || value.find('\0') != std::string::npos)
      throw std::runtime_error("empty value for " + flag);
    if (flag == "--adapter-bootstrap-file") result.bootstrap_file = value;
    else if (flag == "--socket-path") result.socket_path = value;
    else if (flag == "--provider") result.provider = value;
    else if (flag == "--ros-master-uri") result.ros_master_uri = value;
    else if (flag == "--ros-ip") result.ros_ip = value;
    else {
      unsigned int timeout = 0;
      for (const char character : value) {
        if (character < '0' || character > '9' || timeout > 6000)
          throw std::runtime_error("timeout-ms must be an integer between 1 and 60000");
        timeout = timeout * 10 + static_cast<unsigned int>(character - '0');
      }
      if (!timeout || timeout > 60000)
        throw std::runtime_error("timeout-ms must be an integer between 1 and 60000");
      result.timeout_ms = static_cast<int>(timeout);
    }
  }
  if (!result.bootstrap_file.empty()) {
    if (seen.size() != 1) throw std::runtime_error("bootstrap and native arguments cannot be mixed");
    return result;
  }
  ValidateRobotSocketPath(result.socket_path);
  if (result.check) {
    if (seen.count("--provider") || seen.count("--ros-master-uri") || seen.count("--ros-ip"))
      throw std::runtime_error("check accepts only socket-path and timeout-ms");
  } else {
    if (seen.count("--timeout-ms")) throw std::runtime_error("timeout-ms requires check");
    if (result.provider != native_provider) throw std::runtime_error("provider must match this native robot server");
  }
  return result;
}

// Walk existing ancestors without following symbolic links. Only missing
// directories are created, and the direct socket parent must already be private.
inline int OpenPrivateRobotSocketParent(const std::string &path) {
  ValidateRobotSocketPath(path);
  int directory = open("/", O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (directory < 0) throw std::runtime_error("cannot open socket root");
  try {
    std::size_t start = 1;
    for (;;) {
      const auto end = path.find('/', start);
      if (end == std::string::npos) break;
      const auto component = path.substr(start, end - start);
      int next = openat(directory, component.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
      if (next < 0 && errno == ENOENT) {
        if (mkdirat(directory, component.c_str(), 0700) && errno != EEXIST)
          throw std::runtime_error("cannot create private socket directory");
        next = openat(directory, component.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
      }
      if (next < 0) throw std::runtime_error("socket ancestor is unavailable or contains a symbolic link");
      close(directory); directory = next; start = end + 1;
    }
    struct stat parent{};
    if (fstat(directory, &parent) || parent.st_uid != geteuid() || (parent.st_mode & 0777) != 0700)
      throw std::runtime_error("socket parent must be owned by this user with mode 0700");
    return directory;
  } catch (...) { close(directory); throw; }
}
inline void EnsurePrivateRobotSocketParent(const std::string &path) {
  close(OpenPrivateRobotSocketParent(path));
}

// gRPC unlinks its original bind path without checking the inode. Bind only in
// this invocation's private directory and atomically hardlink that same socket
// to the public endpoint. No proxy, additional socket or transport thread exists.
class RobotSocketOwner {
public:
  explicit RobotSocketOwner(std::string path) : path_(std::move(path)) {
    parent_ = OpenPrivateRobotSocketParent(path_);
    basename_ = path_.substr(path_.find_last_of('/') + 1);
    try {
      struct stat current{};
      if (!fstatat(parent_, basename_.c_str(), &current, AT_SYMLINK_NOFOLLOW) || errno != ENOENT)
        throw std::runtime_error("robot socket path already exists or cannot be inspected");
      const auto pattern = "/proc/self/fd/" + std::to_string(parent_) + "/.robot-grpc-XXXXXX";
      std::vector<char> writable(pattern.begin(), pattern.end()); writable.push_back('\0');
      if (!mkdtemp(writable.data())) throw std::runtime_error("cannot create private gRPC binding directory");
      private_name_ = std::string(writable.data()).substr(pattern.find_last_of('/') + 1);
      if (fstatat(parent_, private_name_.c_str(), &current, AT_SYMLINK_NOFOLLOW) || !S_ISDIR(current.st_mode))
        throw std::runtime_error("private gRPC directory is unavailable");
      private_device_ = current.st_dev; private_inode_ = current.st_ino;
      private_ = openat(parent_, private_name_.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
      if (private_ < 0) throw std::runtime_error("cannot open private gRPC binding directory");
      struct stat opened{};
      if (fstat(private_, &opened) || opened.st_dev != private_device_ || opened.st_ino != private_inode_ ||
          opened.st_uid != geteuid() || (opened.st_mode & 0777) != 0700)
        throw std::runtime_error("private gRPC directory identity changed");
      binding_path_ = "/proc/self/fd/" + std::to_string(private_) + "/socket";
    } catch (...) { Cleanup(); throw; }
  }
  RobotSocketOwner(const RobotSocketOwner &) = delete;
  RobotSocketOwner &operator=(const RobotSocketOwner &) = delete;
  const std::string &BindingPath() const { return binding_path_; }
  void RecordBoundSocket() {
    struct stat current{};
    if (fstatat(private_, "socket", &current, AT_SYMLINK_NOFOLLOW) || !S_ISSOCK(current.st_mode) || current.st_uid != geteuid())
      throw std::runtime_error("robot server did not create its Unix socket");
    device_ = current.st_dev; inode_ = current.st_ino; bound_ = true;
    if (fchmodat(private_, "socket", 0600, 0)) throw std::runtime_error("cannot protect robot UDS");
    // Both directory fds refer to the same filesystem. linkat never overwrites
    // a foreign file or socket that appeared after the initial absence check.
    if (linkat(private_, "socket", parent_, basename_.c_str(), 0))
      throw std::runtime_error("cannot claim robot socket path without replacing it");
    owned_ = true;
  }
  ~RobotSocketOwner() { Cleanup(); }
private:
  void Cleanup() noexcept {
    struct stat current{};
    if (owned_ && !fstatat(parent_, basename_.c_str(), &current, AT_SYMLINK_NOFOLLOW) && S_ISSOCK(current.st_mode) &&
        current.st_dev == device_ && current.st_ino == inode_)
      unlinkat(parent_, basename_.c_str(), 0);
    if (bound_ && private_ >= 0 && !fstatat(private_, "socket", &current, AT_SYMLINK_NOFOLLOW) &&
        S_ISSOCK(current.st_mode) && current.st_dev == device_ && current.st_ino == inode_)
      unlinkat(private_, "socket", 0);
    if (private_ >= 0) { close(private_); private_ = -1; }
    if (parent_ >= 0) {
      if (private_inode_ && !fstatat(parent_, private_name_.c_str(), &current, AT_SYMLINK_NOFOLLOW) &&
          S_ISDIR(current.st_mode) && current.st_dev == private_device_ && current.st_ino == private_inode_)
        unlinkat(parent_, private_name_.c_str(), AT_REMOVEDIR);
      close(parent_); parent_ = -1;
    }
  }
  std::string path_, basename_, private_name_, binding_path_;
  int parent_ = -1, private_ = -1;
  dev_t device_{};
  ino_t inode_{};
  dev_t private_device_{};
  ino_t private_inode_{};
  bool bound_ = false, owned_ = false;
};
} // namespace xgc2_ros1_robot_adapter
