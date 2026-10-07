#include "xgc2_ros1_robot_adapter/async_ros_services.hpp"

#include <ares.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <map>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

namespace xgc2_ros1_robot_adapter {
namespace {
constexpr std::size_t maximum_frame = 1024 * 1024;
void u32(std::string &bytes, std::uint32_t value) {
  for (unsigned i = 0; i != 4; ++i) bytes.push_back((value >> (i * 8)) & 255);
}
std::uint32_t read32(const char *bytes) {
  std::uint32_t result = 0;
  for (unsigned i = 0; i != 4; ++i)
    result |= std::uint32_t(static_cast<unsigned char>(bytes[i])) << (i * 8);
  return result;
}
std::string framed(const std::string &bytes) {
  std::string result;
  u32(result, bytes.size());
  result += bytes;
  return result;
}
std::string xmlEscape(const std::string &value) {
  std::string result;
  for (char c : value) {
    switch (c) {
    case '&': result += "&amp;"; break;
    case '<': result += "&lt;"; break;
    case '>': result += "&gt;"; break;
    default: result += c;
    }
  }
  return result;
}
struct Address {
  std::string host, path;
  std::uint16_t port = 0;
};
bool parseAddress(const std::string &uri, const std::string &scheme, Address &out) {
  const auto prefix = scheme + "://";
  if (uri.compare(0, prefix.size(), prefix)) return false;
  auto end = uri.find('/', prefix.size());
  const auto authority = uri.substr(prefix.size(), end - prefix.size());
  out.path = end == std::string::npos ? "/" : uri.substr(end);
  auto colon = authority.rfind(':');
  if (colon == std::string::npos || colon == 0) return false;
  out.host = authority.substr(0, colon);
  if (out.host.front() == '[' && out.host.back() == ']')
    out.host = out.host.substr(1, out.host.size() - 2);
  const auto port = authority.substr(colon + 1);
  if (port.empty() || port.size() > 5 || port.find_first_not_of("0123456789") != std::string::npos)
    return false;
  unsigned n = std::stoul(port);
  if (!n || n > 65535) return false;
  out.port = n;
  return !out.host.empty() && out.host.size() <= 253;
}
} // namespace

struct AsyncRosServices::Impl {
  enum class Stage { ResolveMaster, ConnectMaster, WriteMaster, ReadMaster,
                     ResolveService, ConnectService, WriteHeader, ReadHeader,
                     WriteRequest, ReadResponse };
  struct Call {
    Impl *owner = nullptr;
    Request request;
    std::string resolver_servers;
    Completion completion;
    Stage stage = Stage::ResolveMaster;
    Address address;
    ares_channel resolver = nullptr;
    int fd = -1;
    std::vector<sockaddr_storage> addresses;
    std::size_t next_address = 0;
    std::string send, received;
    std::size_t sent = 0;
    bool resolve_done = false, request_sent = false, done = false;
    bool ipv6_query = false;
    bool master_keepalive = false;
    Result result;
    ~Call() {
      if (resolver) ares_destroy(resolver);
      if (fd >= 0) close(fd);
    }
    void finish(Outcome outcome, std::string detail, std::string response = {}) {
      if (done) return;
      done = true;
      result = {request_sent && outcome != Outcome::Response ? Outcome::Unknown : outcome,
                std::move(response), std::move(detail)};
      if (fd >= 0 && outcome == Outcome::Response && master_keepalive &&
          !request.master_xml.empty() && owner->master_fd < 0) {
        owner->master_fd = fd; owner->master_uri = request.master_uri; fd = -1;
      }
      if (fd >= 0) { close(fd); fd = -1; }
      // ares callbacks only mutate this still-live Call. Destroying the channel
      // cancels UDP/TCP queries; it creates no resolver worker to outlive us.
      if (resolver) { auto channel = resolver; resolver = nullptr; ares_destroy(channel); }
    }
    static void resolved(void *arg, int status, int, hostent *host) {
      auto &call = *static_cast<Call *>(arg);
      if (call.done) return;
      if (status == ARES_SUCCESS && host) {
        for (auto p = host->h_addr_list; *p; ++p) {
          sockaddr_storage address{};
          if (host->h_addrtype == AF_INET) {
            auto &v4 = reinterpret_cast<sockaddr_in &>(address);
            v4.sin_family = AF_INET;
            v4.sin_port = htons(call.address.port);
            std::memcpy(&v4.sin_addr, *p, sizeof(v4.sin_addr));
          } else if (host->h_addrtype == AF_INET6) {
            auto &v6 = reinterpret_cast<sockaddr_in6 &>(address);
            v6.sin6_family = AF_INET6;
            v6.sin6_port = htons(call.address.port);
            std::memcpy(&v6.sin6_addr, *p, sizeof(v6.sin6_addr));
          } else continue;
          call.addresses.push_back(address);
        }
      }
      call.resolve_done = true;
    }
    void resolve(bool master) {
      stage = master ? Stage::ResolveMaster : Stage::ResolveService;
      addresses.clear(); next_address = 0; resolve_done = false; ipv6_query = false;
      sockaddr_storage address{};
      auto &v4 = reinterpret_cast<sockaddr_in &>(address);
      auto &v6 = reinterpret_cast<sockaddr_in6 &>(address);
      if (inet_pton(AF_INET, this->address.host.c_str(), &v4.sin_addr) == 1) {
        v4.sin_family = AF_INET; v4.sin_port = htons(this->address.port);
        addresses.push_back(address); resolve_done = true;
      } else if (inet_pton(AF_INET6, this->address.host.c_str(), &v6.sin6_addr) == 1) {
        v6.sin6_family = AF_INET6; v6.sin6_port = htons(this->address.port);
        addresses.push_back(address); resolve_done = true;
      } else {
        if (ares_init(&resolver) != ARES_SUCCESS) {
          finish(Outcome::NotSent, "DNS channel initialization failed"); return;
        }
        if (!resolver_servers.empty() && ares_set_servers_ports_csv(resolver, resolver_servers.c_str()) != ARES_SUCCESS) {
          finish(Outcome::NotSent, "invalid resolver server"); return;
        }
        ares_gethostbyname(resolver, this->address.host.c_str(), AF_INET, resolved, this);
      }
    }
    void connectNext(bool master) {
      if (fd >= 0) { close(fd); fd = -1; }
      while (next_address < addresses.size()) {
        auto &address = addresses[next_address++];
        fd = socket(address.ss_family, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd < 0) continue;
        const int no_delay = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay));
        const socklen_t size = address.ss_family == AF_INET ? sizeof(sockaddr_in) : sizeof(sockaddr_in6);
        const int result = ::connect(fd, reinterpret_cast<sockaddr *>(&address), size);
        if (!result || errno == EINPROGRESS) {
          stage = master ? Stage::ConnectMaster : Stage::ConnectService;
          return;
        }
        close(fd); fd = -1;
      }
      finish(Outcome::NotSent, "connection failed");
    }
    void prepareSend(std::string bytes, Stage next) {
      send = std::move(bytes); sent = 0; received.clear(); stage = next;
    }
    void connected() {
      int error = 0; socklen_t size = sizeof(error);
      if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &size) || error) {
        connectNext(stage == Stage::ConnectMaster); return;
      }
      if (stage == Stage::ConnectMaster) {
        const auto body = !request.master_xml.empty() ? request.master_xml : "<?xml version=\"1.0\"?><methodCall><methodName>lookupService</methodName><params>"
            "<param><value><string>" + xmlEscape(request.caller_id) + "</string></value></param>"
            "<param><value><string>" + xmlEscape(request.service) + "</string></value></param>"
            "</params></methodCall>";
        prepareSend("POST " + address.path + " HTTP/1.1\r\nHost: " + address.host +
                    "\r\nContent-Type: text/xml\r\nConnection: " +
                    (request.master_xml.empty() ? "close" : "keep-alive") + "\r\nContent-Length: " +
                    std::to_string(body.size()) + "\r\n\r\n" + body, Stage::WriteMaster);
      } else {
        std::string header;
        for (const auto &field : {"callerid=" + request.caller_id, "service=" + request.service,
                                  "md5sum=" + request.md5, "type=" + request.type, std::string("persistent=0")})
          header += framed(field);
        prepareSend(framed(header), Stage::WriteHeader);
      }
    }
    void write() {
      while (sent < send.size()) {
        const auto n = ::send(fd, send.data() + sent, send.size() - sent, MSG_NOSIGNAL);
        if (n > 0) {
          if (stage == Stage::WriteRequest) request_sent = true;
          sent += n;
        } else if (n < 0 && errno == EINTR) continue;
        else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
        else { finish(Outcome::NotSent, "socket write failed"); return; }
      }
      stage = stage == Stage::WriteMaster ? Stage::ReadMaster :
              stage == Stage::WriteHeader ? Stage::ReadHeader : Stage::ReadResponse;
      send.clear(); received.clear();
    }
    void parseMaster() {
      auto end = received.find("\r\n\r\n");
      if (end == std::string::npos) return;
      if (received.compare(0, 12, "HTTP/1.1 200") && received.compare(0, 12, "HTTP/1.0 200")) {
        finish(Outcome::NotSent, "ROS master HTTP error"); return;
      }
      auto headers = received.substr(0, end);
      std::transform(headers.begin(), headers.end(), headers.begin(), [](unsigned char c) { return std::tolower(c); });
      master_keepalive = !received.compare(0, 8, "HTTP/1.1") && headers.find("connection: close") == std::string::npos;
      auto length_pos = headers.find("\r\ncontent-length:");
      if (length_pos == std::string::npos) { finish(Outcome::NotSent, "master response lacks length"); return; }
      length_pos += 17;
      while (length_pos < headers.size() && headers[length_pos] == ' ') ++length_pos;
      auto length_end = headers.find("\r\n", length_pos);
      const auto value = headers.substr(length_pos, length_end - length_pos);
      if (value.empty() || value.size() > 7 || value.find_first_not_of("0123456789") != std::string::npos) {
        finish(Outcome::NotSent, "invalid master response length"); return;
      }
      const auto length = std::stoul(value);
      if (length > maximum_frame) { finish(Outcome::NotSent, "master response too large"); return; }
      if (received.size() < end + 4 + length) return;
      const auto xml = received.substr(end + 4, length);
      if (!request.master_xml.empty()) { finish(Outcome::Response, "ROS master response", xml); return; }
      auto value_pos = xml.find("<value>");
      if (value_pos == std::string::npos) { finish(Outcome::NotSent, "invalid XMLRPC response"); return; }
      int offset = value_pos;
      XmlRpc::XmlRpcValue result;
      try {
        if (!result.fromXml(xml, &offset) || result.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            result.size() != 3 || result[0].getType() != XmlRpc::XmlRpcValue::TypeInt || int(result[0]) != 1 ||
            result[2].getType() != XmlRpc::XmlRpcValue::TypeString ||
            !parseAddress(std::string(result[2]), "rosrpc", address)) {
          finish(Outcome::NotSent, "ROS service lookup failed"); return;
        }
      } catch (...) { finish(Outcome::NotSent, "malformed XMLRPC response"); return; }
      close(fd); fd = -1; received.clear(); resolve(false);
    }
    void parseHeader() {
      if (received.size() < 4) return;
      auto length = read32(received.data());
      if (length > maximum_frame) { finish(Outcome::NotSent, "TCPROS header too large"); return; }
      if (received.size() < length + 4) return;
      std::map<std::string, std::string> fields;
      std::size_t pos = 4;
      while (pos < length + 4) {
        if (pos + 4 > length + 4) { finish(Outcome::NotSent, "truncated TCPROS field"); return; }
        const auto n = read32(received.data() + pos); pos += 4;
        if (n > length + 4 - pos) { finish(Outcome::NotSent, "invalid TCPROS field length"); return; }
        const auto field = received.substr(pos, n); pos += n;
        auto eq = field.find('=');
        if (eq == std::string::npos || !fields.emplace(field.substr(0, eq), field.substr(eq + 1)).second) {
          finish(Outcome::NotSent, "invalid TCPROS header field"); return;
        }
      }
      if (fields.count("error") || fields["md5sum"] != request.md5 || fields["type"] != request.type) {
        finish(Outcome::NotSent, "TCPROS service type/MD5 mismatch"); return;
      }
      prepareSend(framed(request.bytes), Stage::WriteRequest);
    }
    void parseResponse() {
      if (received.size() < 5) return;
      auto length = read32(received.data() + 1);
      if (length > maximum_frame) { finish(Outcome::Unknown, "TCPROS response too large"); return; }
      if (received.size() < length + 5) return;
      if (received[0] != 1) { finish(Outcome::Unknown, "native service reported an error"); return; }
      finish(Outcome::Response, "native response", received.substr(5, length));
    }
    void read() {
      char buffer[8192];
      for (;;) {
        const auto n = recv(fd, buffer, sizeof(buffer), 0);
        if (n > 0) {
          if (received.size() + n > maximum_frame + 16384) { finish(Outcome::NotSent, "response exceeds frame limit"); return; }
          received.append(buffer, n);
          const auto previous = stage;
          if (stage == Stage::ReadMaster) parseMaster();
          else if (stage == Stage::ReadHeader) parseHeader();
          else parseResponse();
          if (done || stage != previous) return;
        } else if (n < 0 && errno == EINTR) continue;
        else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
        else { finish(Outcome::NotSent, "socket closed before complete response"); return; }
      }
    }
  };
  std::map<std::uint64_t, std::unique_ptr<Call>> calls;
  // roscpp's sequential management requests reuse one HTTP connection. This
  // caches transport only: every parameter query and registration still reaches
  // the actual ROS master. PX4 discovery calls remain independently multiplexed.
  int master_fd = -1;
  std::string master_uri;
  struct Watch { Call *call; bool dns; };
  std::vector<pollfd> descriptors;
  std::vector<Watch> watches;
  std::vector<std::pair<Completion, Result>> ready;
  std::uint64_t next = 0;
  std::string resolver_servers;
  explicit Impl(std::string servers) : resolver_servers(std::move(servers)) {
    static std::once_flag initialized;
    std::call_once(initialized, [] {
      if (ares_library_init(ARES_LIB_INIT_ALL) != ARES_SUCCESS) throw std::runtime_error("c-ares initialization failed");
    });
  }
  ~Impl() { calls.clear(); if (master_fd >= 0) close(master_fd); }
  void complete() {
    ready.clear();
    for (auto i = calls.begin(); i != calls.end();) {
      if (!i->second->done) { ++i; continue; }
      ready.emplace_back(std::move(i->second->completion), std::move(i->second->result));
      i = calls.erase(i);
    }
    // Callbacks can submit/cancel calls without invalidating iteration.
    for (auto &entry : ready) entry.first(std::move(entry.second));
    ready.clear();
  }
};

AsyncRosServices::AsyncRosServices(std::string servers) : impl_(new Impl(std::move(servers))) {}
AsyncRosServices::~AsyncRosServices() = default;
std::uint64_t AsyncRosServices::Start(Request request, Completion completion) {
  auto call = std::unique_ptr<Impl::Call>(new Impl::Call);
  call->owner = impl_.get();
  call->request = std::move(request); call->resolver_servers = impl_->resolver_servers; call->completion = std::move(completion);
  const auto id = ++impl_->next;
  if (call->request.bytes.size() > maximum_frame || call->request.master_xml.size() > maximum_frame ||
      !parseAddress(call->request.master_uri, "http", call->address))
    call->finish(Outcome::NotSent, "invalid service request or master URI");
  else {
    if (!call->request.master_xml.empty() && impl_->master_fd >= 0) {
      char byte;
      const auto available = recv(impl_->master_fd, &byte, 1, MSG_PEEK | MSG_DONTWAIT);
      if (impl_->master_uri != call->request.master_uri || available >= 0 ||
          (errno != EAGAIN && errno != EWOULDBLOCK)) {
        close(impl_->master_fd); impl_->master_fd = -1;
      } else {
        call->fd = impl_->master_fd; impl_->master_fd = -1;
        call->stage = Impl::Stage::ConnectMaster;
      }
    }
    if (call->fd < 0) call->resolve(true);
  }
  impl_->calls.emplace(id, std::move(call));
  return id;
}
void AsyncRosServices::Cancel(std::uint64_t id) {
  const auto found = impl_->calls.find(id);
  if (found != impl_->calls.end()) found->second->finish(Outcome::Cancelled, "local I/O cancelled");
}
std::size_t AsyncRosServices::pending() const { return impl_->calls.size(); }
void AsyncRosServices::Pump(std::chrono::milliseconds maximum_wait) {
  auto &descriptors = impl_->descriptors;
  auto &watches = impl_->watches;
  descriptors.clear(); watches.clear();
  auto now = Clock::now();
  auto until = now + maximum_wait;
  for (auto &entry : impl_->calls) {
    auto &call = *entry.second;
    if (call.done) continue;
    if (now >= call.request.deadline) { call.finish(Outcome::Deadline, "absolute service deadline expired"); continue; }
    until = std::min(until, call.request.deadline);
    if (call.stage == Impl::Stage::ResolveMaster || call.stage == Impl::Stage::ResolveService) {
      if (call.resolve_done) {
        if (call.addresses.empty() && call.resolver && !call.ipv6_query) {
          call.ipv6_query = true; call.resolve_done = false;
          ares_gethostbyname(call.resolver, call.address.host.c_str(), AF_INET6, Impl::Call::resolved, &call);
        } else if (call.addresses.empty()) { call.finish(Outcome::NotSent, "DNS lookup failed"); continue; }
        else {
          if (call.resolver) { ares_destroy(call.resolver); call.resolver = nullptr; }
          call.connectNext(call.stage == Impl::Stage::ResolveMaster);
        }
      }
    }
    if (call.done) continue;
    if (call.resolver) {
      ares_socket_t sockets[ARES_GETSOCK_MAXNUM];
      const auto bits = ares_getsock(call.resolver, sockets, ARES_GETSOCK_MAXNUM);
      for (int i = 0; i != ARES_GETSOCK_MAXNUM; ++i) {
        short events = 0;
        if (ARES_GETSOCK_READABLE(bits, i)) events |= POLLIN;
        if (ARES_GETSOCK_WRITABLE(bits, i)) events |= POLLOUT;
        if (events) { descriptors.push_back({sockets[i], events, 0}); watches.push_back({&call, true}); }
      }
      timeval limit{0, 10000}, timeout{};
      ares_timeout(call.resolver, &limit, &timeout);
      until = std::min(until, now + std::chrono::microseconds(timeout.tv_sec * 1000000 + timeout.tv_usec));
    } else if (call.fd >= 0) {
      const bool reading = call.stage == Impl::Stage::ReadMaster || call.stage == Impl::Stage::ReadHeader || call.stage == Impl::Stage::ReadResponse;
      descriptors.push_back({call.fd, short(reading ? POLLIN : POLLOUT), 0}); watches.push_back({&call, false});
    }
  }
  auto wait = std::chrono::duration_cast<std::chrono::milliseconds>(until - Clock::now()).count();
  poll(descriptors.data(), descriptors.size(), std::max<long long>(0, wait));
  for (std::size_t i = 0; i < descriptors.size(); ++i) {
    auto &call = *watches[i].call; auto &fd = descriptors[i];
    if (call.done || !fd.revents) continue;
    if (Clock::now() >= call.request.deadline) { call.finish(Outcome::Deadline, "absolute service deadline expired"); continue; }
    if (watches[i].dns) {
      if (call.resolver) ares_process_fd(call.resolver, fd.revents & (POLLIN | POLLERR | POLLHUP) ? fd.fd : ARES_SOCKET_BAD,
                                       fd.revents & POLLOUT ? fd.fd : ARES_SOCKET_BAD);
    } else if (call.stage == Impl::Stage::ConnectMaster || call.stage == Impl::Stage::ConnectService) call.connected();
    else if (call.stage == Impl::Stage::WriteMaster || call.stage == Impl::Stage::WriteHeader || call.stage == Impl::Stage::WriteRequest) call.write();
    else call.read();
  }
  for (auto &entry : impl_->calls) {
    auto &call = *entry.second;
    if (!call.done && Clock::now() >= call.request.deadline) call.finish(Outcome::Deadline, "absolute service deadline expired");
    else if (!call.done && call.resolver) ares_process_fd(call.resolver, ARES_SOCKET_BAD, ARES_SOCKET_BAD);
  }
  impl_->complete();
}
} // namespace xgc2_ros1_robot_adapter
