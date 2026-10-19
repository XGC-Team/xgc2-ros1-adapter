#include "xgc2_ros1_robot_adapter/robot_server.hpp"
#include <algorithm>
#include <cstring>
#include <set>
#include <stdexcept>
#include <sys/un.h>

namespace xgc2_ros1_robot_adapter {
namespace {
bool identifierCharacter(unsigned char character) {
  return (character >= 'a' && character <= 'z') ||
         (character >= 'A' && character <= 'Z') ||
         (character >= '0' && character <= '9') || character == '.' ||
         character == '_' || character == ':' || character == '-';
}
// The characters of a ROS name; everything else becomes an underscore.
std::string RosNameCharacters(const std::string &text) {
  std::string name;
  for (const unsigned char character : text)
    name += (character >= 'a' && character <= 'z') ||
                    (character >= 'A' && character <= 'Z') ||
                    (character >= '0' && character <= '9') || character == '_'
                ? static_cast<char>(character)
                : '_';
  return name;
}
void requireIdentifier(const std::string &flag, const std::string &value) {
  if (!ValidReferenceId(value))
    throw std::invalid_argument(
        flag + " must be 1-128 characters of [A-Za-z0-9._:-]");
}
void requireSocketPath(const std::string &path) {
  if (path.empty() || path.front() != '/' ||
      path.find('\0') != std::string::npos ||
      path.size() >= sizeof(sockaddr_un::sun_path))
    throw std::invalid_argument(
        "--socket-path must be a bounded absolute Unix path");
  std::size_t start = 1;
  for (;;) {
    const auto end = path.find('/', start);
    const auto component =
        path.substr(start, end == std::string::npos ? end : end - start);
    if (component.empty() || component == "." || component == "..")
      throw std::invalid_argument(
          "--socket-path contains an invalid component");
    if (end == std::string::npos)
      break;
    start = end + 1;
  }
}
// The master is addressed as http://host:port[/]; roscpp rejects anything else.
void requireMasterUri(const std::string &uri) {
  static const std::string scheme = "http://";
  if (uri.size() > 2048 || uri.compare(0, scheme.size(), scheme) != 0)
    throw std::invalid_argument("--ros-master-uri must be an http:// URI");
  auto end = uri.find('/', scheme.size());
  const auto authority = uri.substr(
      scheme.size(), end == std::string::npos ? end : end - scheme.size());
  const auto colon = authority.rfind(':');
  const auto port =
      colon == std::string::npos ? std::string() : authority.substr(colon + 1);
  if (colon == std::string::npos || colon == 0 || port.empty() ||
      port.size() > 5 || port.find_first_not_of("0123456789") != std::string::npos ||
      std::stoul(port) == 0 || std::stoul(port) > 65535 ||
      (end != std::string::npos && uri.find_first_not_of('/', end) != std::string::npos))
    throw std::invalid_argument(
        "--ros-master-uri must name host and port, e.g. http://127.0.0.1:11311");
}
} // namespace

bool ValidReferenceId(const std::string &value) {
  return !value.empty() && value.size() <= 128 &&
         std::all_of(value.begin(), value.end(), identifierCharacter);
}

ServerOptions ParseServerOptions(int argc, char **argv,
                                 const std::vector<std::string> &providers) {
  static const std::set<std::string> flags{
      "--socket-path", "--run-id",         "--node-name", "--provider",
      "--target-id",   "--ros-master-uri", "--ros-ip", "--network-bootstrap"};
  ServerOptions options;
  std::set<std::string> seen;
  for (int index = 1; index < argc; ++index) {
    const std::string flag(argv[index]);
    if (!flags.count(flag))
      throw std::invalid_argument("unknown argument: " + flag);
    if (!seen.insert(flag).second)
      throw std::invalid_argument("argument occurs more than once: " + flag);
    if (index + 1 >= argc || std::strncmp(argv[index + 1], "--", 2) == 0)
      throw std::invalid_argument("missing value for " + flag);
    const std::string value(argv[++index]);
    if (value.find('\0') != std::string::npos ||
        (value.empty() && flag != "--ros-ip" && flag != "--network-bootstrap"))
      throw std::invalid_argument("empty value for " + flag);
    if (flag == "--socket-path")
      options.socket_path = value;
    else if (flag == "--run-id")
      options.run_id = value;
    else if (flag == "--node-name")
      options.node_name = value;
    else if (flag == "--provider")
      options.provider = value;
    else if (flag == "--target-id")
      options.target_id = value;
    else if (flag == "--ros-master-uri")
      options.ros_master_uri = value;
    else if (flag == "--network-bootstrap")
      options.network_bootstrap = value;
    else
      options.ros_ip = value;
  }
  for (const auto &flag : flags)
    if (flag != "--ros-ip" && flag != "--network-bootstrap" && !seen.count(flag))
      throw std::invalid_argument("missing required argument: " + flag);
  requireSocketPath(options.socket_path);
  requireIdentifier("--run-id", options.run_id);
  requireIdentifier("--target-id", options.target_id);
  if (options.node_name.size() > 128)
    throw std::invalid_argument("--node-name must have at most 128 characters");
  // Two Runs on one ROS master must never share a node name: the master would
  // let the newer node displace the older one.
  if (RosNameCharacters(options.node_name).find(RosNameCharacters(options.run_id)) ==
      std::string::npos)
    throw std::invalid_argument("--node-name must include the --run-id");
  if (std::find(providers.begin(), providers.end(), options.provider) ==
      providers.end())
    throw std::invalid_argument("--provider is not served by this executable: " +
                                options.provider);
  requireMasterUri(options.ros_master_uri);
  if (options.ros_ip.size() > 253)
    throw std::invalid_argument("--ros-ip is too long");
  if (!options.network_bootstrap.empty() &&
      (options.network_bootstrap.front() != '/' || options.network_bootstrap.size() > 4096))
    throw std::invalid_argument("--network-bootstrap must be an absolute file path");
  return options;
}

std::string ServerUsage(const char *program) {
  return std::string("usage: ") + program +
         " --socket-path PATH --run-id ID --node-name NAME --provider TYPE\n"
         "         --target-id ID --ros-master-uri URI [--ros-ip ADDRESS]\n"
         "         [--network-bootstrap PATH]";
}

std::string RosNodeName(const std::string &node_name) {
  std::string name = RosNameCharacters(node_name);
  if (name.empty() || (name.front() >= '0' && name.front() <= '9') ||
      name.front() == '_')
    name.insert(name.begin(), 'n');
  return name;
}
} // namespace xgc2_ros1_robot_adapter
