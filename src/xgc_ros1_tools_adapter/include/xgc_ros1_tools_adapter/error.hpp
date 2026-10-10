#pragma once

#include <stdexcept>
#include <string>
#include <utility>

namespace xgc_ros1_tools_adapter {

class Ros1ToolsError : public std::runtime_error {
 public:
  Ros1ToolsError(std::string code, std::string error_class, std::string message)
      : std::runtime_error(std::move(message)),
        code_(std::move(code)),
        error_class_(std::move(error_class)) {}

  const std::string& code() const noexcept { return code_; }
  const std::string& errorClass() const noexcept { return error_class_; }

 private:
  std::string code_;
  std::string error_class_;
};

[[noreturn]] inline void permanentError(const std::string& code,
                                        const std::string& message) {
  throw Ros1ToolsError(code, "permanent", message);
}

[[noreturn]] inline void transientError(const std::string& code,
                                        const std::string& message) {
  throw Ros1ToolsError(code, "transient", message);
}

[[noreturn]] inline void uncertainError(const std::string& code,
                                        const std::string& message) {
  throw Ros1ToolsError(code, "uncertain", message);
}

[[noreturn]] inline void cancelledError(const std::string& code,
                                        const std::string& message) {
  throw Ros1ToolsError(code, "cancelled", message);
}

[[noreturn]] inline void deadlineError(const std::string& code,
                                       const std::string& message) {
  throw Ros1ToolsError(code, "deadline", message);
}

[[noreturn]] inline void rejectedError(const std::string& code,
                                       const std::string& message) {
  throw Ros1ToolsError(code, "rejected", message);
}

[[noreturn]] inline void resourceExhaustedError(const std::string& code,
                                                const std::string& message) {
  throw Ros1ToolsError(code, "resource-exhausted", message);
}

}  // namespace xgc_ros1_tools_adapter
