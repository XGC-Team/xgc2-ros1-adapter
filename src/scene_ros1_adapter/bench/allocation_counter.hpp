#pragma once

// One benchmark translation unit replaces all global C++ allocation forms.
#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <new>

namespace allocation_counter {
inline std::atomic<bool> enabled{false};
inline std::atomic<std::size_t> count{0};
inline void *allocate(std::size_t size, std::size_t alignment) {
  if (enabled.load(std::memory_order_relaxed))
    count.fetch_add(1, std::memory_order_relaxed);
  void *p = nullptr;
  if (alignment <= alignof(std::max_align_t))
    p = std::malloc(size ? size : 1);
  else if (posix_memalign(&p, alignment, size ? size : 1) != 0)
    p = nullptr;
  if (!p)
    throw std::bad_alloc();
  return p;
}
inline void start() {
  count.store(0);
  enabled.store(true);
}
inline std::size_t stop() {
  enabled.store(false);
  return count.load();
}
} // namespace allocation_counter

void *operator new(std::size_t n) {
  return allocation_counter::allocate(n, alignof(std::max_align_t));
}
void *operator new[](std::size_t n) {
  return allocation_counter::allocate(n, alignof(std::max_align_t));
}
void *operator new(std::size_t n, std::align_val_t a) {
  return allocation_counter::allocate(n, static_cast<std::size_t>(a));
}
void *operator new[](std::size_t n, std::align_val_t a) {
  return allocation_counter::allocate(n, static_cast<std::size_t>(a));
}
void *operator new(std::size_t n, const std::nothrow_t &) noexcept {
  try {
    return ::operator new(n);
  } catch (...) {
    return nullptr;
  }
}
void *operator new[](std::size_t n, const std::nothrow_t &) noexcept {
  try {
    return ::operator new[](n);
  } catch (...) {
    return nullptr;
  }
}
void *operator new(std::size_t n, std::align_val_t a,
                   const std::nothrow_t &) noexcept {
  try {
    return ::operator new(n, a);
  } catch (...) {
    return nullptr;
  }
}
void *operator new[](std::size_t n, std::align_val_t a,
                     const std::nothrow_t &) noexcept {
  try {
    return ::operator new[](n, a);
  } catch (...) {
    return nullptr;
  }
}
void operator delete(void *p) noexcept { std::free(p); }
void operator delete[](void *p) noexcept { std::free(p); }
void operator delete(void *p, std::size_t) noexcept { std::free(p); }
void operator delete[](void *p, std::size_t) noexcept { std::free(p); }
void operator delete(void *p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void *p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void *p, std::size_t, std::align_val_t) noexcept {
  std::free(p);
}
void operator delete[](void *p, std::size_t, std::align_val_t) noexcept {
  std::free(p);
}
void operator delete(void *p, const std::nothrow_t &) noexcept { std::free(p); }
void operator delete[](void *p, const std::nothrow_t &) noexcept {
  std::free(p);
}
void operator delete(void *p, std::align_val_t,
                     const std::nothrow_t &) noexcept {
  std::free(p);
}
void operator delete[](void *p, std::align_val_t,
                       const std::nothrow_t &) noexcept {
  std::free(p);
}
