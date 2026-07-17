#include <gtest/gtest.h>

#include <array>
#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

#include <dirent.h>
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

namespace xgc_px4_multirotor_ros1_adapter {
namespace {

constexpr int kHelperSocketDescriptor = 198;
constexpr int kPollTimeoutMillis = 5000;

struct HelperProcess {
  pid_t process_id = -1;
  int socket_fd = -1;
};

void closeDescriptor(int *descriptor) {
  if (descriptor == nullptr || *descriptor < 0)
    return;
  close(*descriptor);
  *descriptor = -1;
}

bool waitForChild(pid_t process_id, int options, int timeout_millis,
                  int *status) {
  const int iterations = timeout_millis / 10 + 1;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    pid_t waited;
    do {
      waited = waitpid(process_id, status, options | WNOHANG);
    } while (waited < 0 && errno == EINTR);
    if (waited == process_id)
      return true;
    if (waited < 0)
      return false;
    usleep(10000u);
  }
  return false;
}

void terminateHelper(HelperProcess *helper) {
  if (helper == nullptr)
    return;
  closeDescriptor(&helper->socket_fd);
  if (helper->process_id <= 0)
    return;
  kill(helper->process_id, SIGKILL);
  int status = 0;
  pid_t waited;
  do {
    waited = waitpid(helper->process_id, &status, 0);
  } while (waited < 0 && errno == EINTR);
  helper->process_id = -1;
}

class ScopedHelper {
public:
  ~ScopedHelper() { terminateHelper(&helper); }

  HelperProcess helper;
};

std::string helperExecutablePath() {
  std::array<char, PATH_MAX + 1u> executable{};
  const ssize_t size = readlink("/proc/self/exe", executable.data(), PATH_MAX);
  if (size <= 0 || size == PATH_MAX)
    return {};
  executable[static_cast<std::size_t>(size)] = '\0';
  const std::string current(executable.data());
  const std::size_t separator = current.rfind('/');
  if (separator == std::string::npos)
    return {};
  return current.substr(0u, separator + 1u) +
         "xgc_px4_multirotor_ros1_adapter_service_helper";
}

bool spawnHelper(HelperProcess *helper, std::string *error) {
  if (helper == nullptr)
    return false;
  const std::string helper_path = helperExecutablePath();
  if (helper_path.empty() || access(helper_path.c_str(), X_OK) != 0) {
    if (error != nullptr)
      *error = "cannot locate the sibling PX4 service helper";
    return false;
  }

  int sockets[2] = {-1, -1};
  if (socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0,
                 sockets) != 0) {
    if (error != nullptr)
      *error = std::string("socketpair failed: ") + std::strerror(errno);
    return false;
  }
  for (int index = 0; index < 2; ++index) {
    if (sockets[index] != kHelperSocketDescriptor)
      continue;
    const int replacement =
        fcntl(sockets[index], F_DUPFD_CLOEXEC, kHelperSocketDescriptor + 1);
    if (replacement < 0) {
      if (error != nullptr)
        *error =
            std::string("socket relocation failed: ") + std::strerror(errno);
      close(sockets[0]);
      close(sockets[1]);
      return false;
    }
    close(sockets[index]);
    sockets[index] = replacement;
  }

  posix_spawn_file_actions_t actions;
  int spawn_error = posix_spawn_file_actions_init(&actions);
  const bool actions_initialized = spawn_error == 0;
  if (spawn_error == 0)
    spawn_error = posix_spawn_file_actions_adddup2(&actions, sockets[1],
                                                   kHelperSocketDescriptor);
  if (spawn_error == 0)
    spawn_error = posix_spawn_file_actions_addclose(&actions, sockets[0]);
  if (spawn_error == 0)
    spawn_error = posix_spawn_file_actions_addclose(&actions, sockets[1]);

  pid_t child = -1;
  if (spawn_error == 0) {
    const std::string descriptor = std::to_string(kHelperSocketDescriptor);
    std::array<char *, 6u> arguments{const_cast<char *>(helper_path.c_str()),
                                     const_cast<char *>(descriptor.c_str()),
                                     const_cast<char *>("/test/arming"),
                                     const_cast<char *>("/test/set_mode"),
                                     const_cast<char *>("/test/command"),
                                     nullptr};
    spawn_error = posix_spawn(&child, helper_path.c_str(), &actions, nullptr,
                              arguments.data(), environ);
  }
  if (actions_initialized)
    posix_spawn_file_actions_destroy(&actions);
  close(sockets[1]);
  if (spawn_error != 0) {
    close(sockets[0]);
    if (error != nullptr)
      *error = std::string("posix_spawn failed: ") + std::strerror(spawn_error);
    return false;
  }

  helper->process_id = child;
  helper->socket_fd = sockets[0];
  return true;
}

bool descriptorTarget(pid_t process_id, int descriptor, std::string *target) {
  const std::string path = "/proc/" + std::to_string(process_id) + "/fd/" +
                           std::to_string(descriptor);
  std::array<char, PATH_MAX + 1u> value{};
  const ssize_t size = readlink(path.c_str(), value.data(), PATH_MAX);
  if (size < 0)
    return false;
  value[static_cast<std::size_t>(size)] = '\0';
  if (target != nullptr)
    *target = value.data();
  return true;
}

bool processIsLive(pid_t process_id) {
  const std::string path = "/proc/" + std::to_string(process_id) + "/stat";
  const int descriptor = open(path.c_str(), O_RDONLY | O_CLOEXEC);
  if (descriptor < 0)
    return false;
  std::array<char, 1024u> contents{};
  ssize_t size;
  do {
    size = read(descriptor, contents.data(), contents.size());
  } while (size < 0 && errno == EINTR);
  close(descriptor);
  if (size <= 0)
    return false;

  const std::string stat(contents.data(), static_cast<std::size_t>(size));
  const std::size_t command_end = stat.rfind(')');
  if (command_end == std::string::npos || command_end + 2u >= stat.size())
    return false;
  const char state = stat[command_end + 2u];
  return state != 'Z' && state != 'X' && state != 'x';
}

bool processHasDescriptorTarget(pid_t process_id, const std::string &target,
                                bool *found) {
  if (found == nullptr)
    return false;
  *found = false;
  const std::string directory_path =
      "/proc/" + std::to_string(process_id) + "/fd";
  DIR *directory = opendir(directory_path.c_str());
  if (directory == nullptr)
    return false;

  bool complete = true;
  for (;;) {
    errno = 0;
    const dirent *entry = readdir(directory);
    if (entry == nullptr) {
      complete = errno == 0;
      break;
    }
    errno = 0;
    char *end = nullptr;
    const long parsed = std::strtol(entry->d_name, &end, 10);
    if (errno != 0 || end == entry->d_name || *end != '\0' || parsed < 0L ||
        parsed > INT_MAX) {
      continue;
    }
    std::string descriptor_target;
    if (descriptorTarget(process_id, static_cast<int>(parsed),
                         &descriptor_target) &&
        descriptor_target == target) {
      *found = true;
      break;
    }
  }
  if (closedir(directory) != 0)
    complete = false;
  return complete;
}

bool helperLifecycleReady(pid_t process_id,
                          const std::string &inherited_target) {
  const int iterations = kPollTimeoutMillis / 10 + 1;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    if (!processIsLive(process_id))
      return false;
    bool inherited_target_found = false;
    if (processHasDescriptorTarget(process_id, inherited_target,
                                   &inherited_target_found) &&
        !inherited_target_found) {
      std::string control_target;
      return descriptorTarget(process_id, kHelperSocketDescriptor,
                              &control_target) &&
             control_target.compare(0u, 8u, "socket:[") == 0 &&
             processIsLive(process_id);
    }
    usleep(10000u);
  }
  return false;
}

bool writeAll(int descriptor, const void *data, std::size_t size) {
  const auto *cursor = static_cast<const std::uint8_t *>(data);
  while (size > 0u) {
    const ssize_t written = write(descriptor, cursor, size);
    if (written > 0) {
      cursor += written;
      size -= static_cast<std::size_t>(written);
      continue;
    }
    if (written < 0 && errno == EINTR)
      continue;
    return false;
  }
  return true;
}

bool readAll(int descriptor, void *data, std::size_t size) {
  auto *cursor = static_cast<std::uint8_t *>(data);
  while (size > 0u) {
    const ssize_t received = read(descriptor, cursor, size);
    if (received > 0) {
      cursor += received;
      size -= static_cast<std::size_t>(received);
      continue;
    }
    if (received < 0 && errno == EINTR)
      continue;
    return false;
  }
  return true;
}

class ScopedSubreaper {
public:
  bool enable() {
    if (prctl(PR_GET_CHILD_SUBREAPER, &previous_) != 0)
      return false;
    if (prctl(PR_SET_CHILD_SUBREAPER, 1) != 0)
      return false;
    enabled_ = true;
    return true;
  }

  ~ScopedSubreaper() {
    if (enabled_)
      prctl(PR_SET_CHILD_SUBREAPER, previous_);
  }

private:
  int previous_ = 0;
  bool enabled_ = false;
};

TEST(Px4ServiceHelperLifecycle, ClosesUnrelatedInheritedDescriptors) {
  int inherited_pipe[2] = {-1, -1};
  ASSERT_EQ(0, pipe(inherited_pipe));
  std::string inherited_target;
  ASSERT_TRUE(descriptorTarget(getpid(), inherited_pipe[0], &inherited_target));

  ScopedHelper helper;
  std::string error;
  ASSERT_TRUE(spawnHelper(&helper.helper, &error)) << error;
  EXPECT_TRUE(helperLifecycleReady(helper.helper.process_id, inherited_target));
  close(inherited_pipe[0]);
  close(inherited_pipe[1]);
}

TEST(Px4ServiceHelperLifecycle, DiesWhenItsSpawningParentExits) {
  ScopedSubreaper subreaper;
  ASSERT_TRUE(subreaper.enable());

  int report_pipe[2] = {-1, -1};
  ASSERT_EQ(0, pipe2(report_pipe, O_CLOEXEC));
  const pid_t launcher = fork();
  ASSERT_GE(launcher, 0);
  if (launcher == 0) {
    close(report_pipe[0]);
    int inherited_pipe[2] = {-1, -1};
    HelperProcess helper;
    std::string error;
    int failure = 0;
    std::string inherited_target;
    if (pipe(inherited_pipe) != 0 ||
        !descriptorTarget(getpid(), inherited_pipe[0], &inherited_target)) {
      failure = 1;
    } else if (!spawnHelper(&helper, &error) ||
               !helperLifecycleReady(helper.process_id, inherited_target)) {
      failure = 2;
    } else if (kill(helper.process_id, SIGSTOP) != 0) {
      failure = 3;
    } else {
      int stopped_status = 0;
      pid_t waited;
      do {
        waited = waitpid(helper.process_id, &stopped_status, WUNTRACED);
      } while (waited < 0 && errno == EINTR);
      if (waited != helper.process_id || !WIFSTOPPED(stopped_status))
        failure = 4;
    }
    closeDescriptor(&inherited_pipe[0]);
    closeDescriptor(&inherited_pipe[1]);

    const std::array<std::int64_t, 2u> report{
        static_cast<std::int64_t>(helper.process_id), failure};
    const bool reported =
        writeAll(report_pipe[1], report.data(), sizeof(report));
    if (failure != 0 || !reported) {
      terminateHelper(&helper);
      _exit(1);
    }
    // _exit closes the control socket, but the helper is stopped and can only
    // terminate promptly if its already-installed parent-death fence fires.
    _exit(0);
  }

  close(report_pipe[1]);
  std::array<std::int64_t, 2u> report{};
  ASSERT_TRUE(readAll(report_pipe[0], report.data(), sizeof(report)));
  close(report_pipe[0]);

  int launcher_status = 0;
  pid_t launcher_waited;
  do {
    launcher_waited = waitpid(launcher, &launcher_status, 0);
  } while (launcher_waited < 0 && errno == EINTR);
  ASSERT_EQ(launcher, launcher_waited);
  ASSERT_TRUE(WIFEXITED(launcher_status));
  ASSERT_EQ(0, WEXITSTATUS(launcher_status));
  ASSERT_EQ(0, report[1]);

  const pid_t helper_pid = static_cast<pid_t>(report[0]);
  ASSERT_GT(helper_pid, 0);
  int helper_status = 0;
  if (!waitForChild(helper_pid, 0, kPollTimeoutMillis, &helper_status)) {
    kill(helper_pid, SIGKILL);
    waitpid(helper_pid, &helper_status, 0);
    FAIL() << "helper survived its spawning parent";
  }
  EXPECT_TRUE(WIFSIGNALED(helper_status));
  if (WIFSIGNALED(helper_status)) {
    EXPECT_EQ(SIGKILL, WTERMSIG(helper_status));
  }
}

} // namespace
} // namespace xgc_px4_multirotor_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
