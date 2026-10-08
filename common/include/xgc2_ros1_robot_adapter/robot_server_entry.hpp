#pragma once

#include <cerrno>
#include <cstdlib>
#include <cstring>
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
  std::string bootstrap_file, socket_path, provider, target_id, ros_master_uri,
      ros_ip;
  bool check = false;
  int timeout_ms = 2000;
};

inline bool ValidRuntimeReferenceId(const std::string &value) {
  if (value.empty() || value.size() > 128)
    return false;
  for (const unsigned char character : value)
    if (!((character >= 'a' && character <= 'z') ||
          (character >= 'A' && character <= 'Z') ||
          (character >= '0' && character <= '9') || character == '.' ||
          character == '_' || character == ':' || character == '-'))
      return false;
  return true;
}

inline void ValidateRobotSocketPath(const std::string &path) {
  if (path.empty() || path.front() != '/' ||
      path.find('\0') != std::string::npos ||
      path.size() >= sizeof(sockaddr_un::sun_path))
    throw std::runtime_error(
        "socket path must be a bounded absolute Unix path");
  std::size_t start = 1;
  for (;;) {
    const auto end = path.find('/', start);
    const auto component =
        path.substr(start, end == std::string::npos ? end : end - start);
    if (component.empty() || component == "." || component == "..")
      throw std::runtime_error("socket path contains an invalid component");
    if (end == std::string::npos)
      break;
    start = end + 1;
  }
}

inline RobotServerArguments
ParseRobotServerArguments(int argc, char **argv,
                          const std::string &native_provider) {
  RobotServerArguments result;
  std::set<std::string> seen;
  for (int index = 1; index < argc; ++index) {
    const std::string flag(argv[index]);
    if (flag != "--adapter-bootstrap-file" && flag != "--socket-path" &&
        flag != "--provider" && flag != "--target-id" &&
        flag != "--ros-master-uri" && flag != "--ros-ip" && flag != "--check" &&
        flag != "--timeout-ms")
      throw std::runtime_error("unknown argument: " + flag);
    if (!seen.insert(flag).second)
      throw std::runtime_error("argument occurs more than once: " + flag);
    if (flag == "--check") {
      result.check = true;
      continue;
    }
    if (index + 1 >= argc ||
        std::string(argv[index + 1]).compare(0, 2, "--") == 0)
      throw std::runtime_error("missing value for " + flag);
    const std::string value(argv[++index]);
    if ((value.empty() && flag != "--ros-ip") ||
        value.find('\0') != std::string::npos)
      throw std::runtime_error("empty value for " + flag);
    if (flag == "--adapter-bootstrap-file")
      result.bootstrap_file = value;
    else if (flag == "--socket-path")
      result.socket_path = value;
    else if (flag == "--provider")
      result.provider = value;
    else if (flag == "--target-id")
      result.target_id = value;
    else if (flag == "--ros-master-uri")
      result.ros_master_uri = value;
    else if (flag == "--ros-ip")
      result.ros_ip = value;
    else {
      unsigned int timeout = 0;
      for (const char character : value) {
        if (character < '0' || character > '9' || timeout > 6000)
          throw std::runtime_error(
              "timeout-ms must be an integer between 1 and 60000");
        timeout = timeout * 10 + static_cast<unsigned int>(character - '0');
      }
      if (!timeout || timeout > 60000)
        throw std::runtime_error(
            "timeout-ms must be an integer between 1 and 60000");
      result.timeout_ms = static_cast<int>(timeout);
    }
  }
  if (!result.bootstrap_file.empty()) {
    if (seen.size() != 1)
      throw std::runtime_error(
          "bootstrap and native arguments cannot be mixed");
    return result;
  }
  ValidateRobotSocketPath(result.socket_path);
  if (result.check) {
    if (!ValidRuntimeReferenceId(result.target_id))
      throw std::runtime_error("check requires a bounded target-id");
    if (seen.count("--provider") || seen.count("--ros-master-uri") ||
        seen.count("--ros-ip"))
      throw std::runtime_error(
          "check accepts only socket-path, target-id and timeout-ms");
  } else {
    if (seen.count("--timeout-ms"))
      throw std::runtime_error("timeout-ms requires check");
    if (!ValidRuntimeReferenceId(result.target_id))
      throw std::runtime_error("target-id must be a bounded XRPC identifier");
    if (result.provider != native_provider)
      throw std::runtime_error("provider must match this native robot server");
  }
  return result;
}

// Walk existing ancestors without following symbolic links. Only missing
// directories are created, and the direct socket parent must already be
// private.
inline int OpenPrivateRobotSocketParent(const std::string &path) {
  ValidateRobotSocketPath(path);
  int directory = open("/", O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (directory < 0)
    throw std::runtime_error("cannot open socket root");
  try {
    std::size_t start = 1;
    for (;;) {
      const auto end = path.find('/', start);
      if (end == std::string::npos)
        break;
      const auto component = path.substr(start, end - start);
      int next = openat(directory, component.c_str(),
                        O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
      if (next < 0 && errno == ENOENT) {
        if (mkdirat(directory, component.c_str(), 0700) && errno != EEXIST)
          throw std::runtime_error("cannot create private socket directory");
        next = openat(directory, component.c_str(),
                      O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
      }
      if (next < 0)
        throw std::runtime_error(
            "socket ancestor is unavailable or contains a symbolic link");
      close(directory);
      directory = next;
      start = end + 1;
    }
    struct stat parent {};
    if (fstat(directory, &parent) || parent.st_uid != geteuid() ||
        (parent.st_mode & 0777) != 0700)
      throw std::runtime_error(
          "socket parent must be owned by this user with mode 0700");
    return directory;
  } catch (...) {
    close(directory);
    throw;
  }
}
inline void EnsurePrivateRobotSocketParent(const std::string &path) {
  close(OpenPrivateRobotSocketParent(path));
}

} // namespace xgc2_ros1_robot_adapter
