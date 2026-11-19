#pragma once
#include <atomic>
#include <csignal>
#include <cstdint>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace xgc::recording {

// The frozen world endpoint owns this adjacent read-only capability. Verify
// its filesystem identity once, then leave it bound to that exact generation.
class XSimReceiptClock {
 public:
  void open(const std::string &world) {
    struct stat endpoint {}, file {};
    if (world.empty() || world.front() != '/' || lstat(world.c_str(), &endpoint) ||
        !S_ISSOCK(endpoint.st_mode))
      throw std::runtime_error("receipt clock world endpoint is unavailable");
    const int fd = ::open((world + ".clock-v1").c_str(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC | O_NONBLOCK);
    if (fd < 0) throw std::runtime_error("receipt clock capability is unavailable");
    if (fstat(fd, &file) || !S_ISREG(file.st_mode) || file.st_uid != endpoint.st_uid ||
        (file.st_mode & (S_IWGRP | S_IWOTH)) || file.st_size != sizeof(Fields)) {
      close(fd);
      throw std::runtime_error("receipt clock capability has an invalid identity");
    }
    auto *mapping = mmap(nullptr, sizeof(Fields), PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (mapping == MAP_FAILED) throw std::runtime_error("could not map receipt clock capability");
    fields_ = static_cast<const Fields *>(mapping);
    if (fields_->socket_inode != endpoint.st_ino || !stamp() || (stamp() & retired))
      throw std::runtime_error("receipt clock does not belong to the running world");
  }

  std::uint64_t stamp() const noexcept {
    return fields_ ? __atomic_load_n(&fields_->stamp, __ATOMIC_ACQUIRE) : 0;
  }

  ~XSimReceiptClock() {
    if (fields_) munmap(const_cast<Fields *>(fields_), sizeof(Fields));
  }

  static constexpr std::uint64_t retired = std::uint64_t{1} << 63;

 private:
  struct Fields { std::uint64_t socket_inode, stamp; };
  static_assert(__atomic_always_lock_free(sizeof(std::uint64_t), nullptr));
  const Fields *fields_ = nullptr;
};

} // namespace xgc::recording
