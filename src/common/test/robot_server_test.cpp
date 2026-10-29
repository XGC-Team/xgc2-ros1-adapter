// The type server core against a fake RosBinding: real gRPC over a Unix
// socket, no ROS. The fake binding scripts the master and records what the
// server does to its robots, so the lifecycle can be asserted exactly.
#include "xgc/robot/v1/server.grpc.pb.h"
#include "xgc2/xrpc/grpc.hpp"
#include "xgc2_ros1_robot_adapter/core_silence.hpp"
#include "xgc2_ros1_robot_adapter/robot_server.hpp"
#include "xgc2_ros1_robot_adapter/async_ros_services.hpp"
#include "xgc2_ros1_robot_adapter/master_retry.hpp"
#include <openssl/pem.h>
#include <openssl/x509v3.h>
#include <arpa/inet.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <dirent.h>
#include <fcntl.h>
#include <future>
#include <gtest/gtest.h>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <netinet/in.h>
#include <set>
#include <stdexcept>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

namespace xgc2_ros1_robot_adapter {
namespace {
namespace wire = xgc::robot::v1;
namespace operation = xgc::adapter::v1;
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;
using xgc2::xrpc::GrpcClientCall;
using xgc2::xrpc::GrpcClock;

const std::string kTestProcessOwner(32, 'p');

void prepareOwnerInput(const std::string &value) {
  int descriptors[2];
  if (::pipe(descriptors) != 0) throw std::runtime_error("test owner pipe");
  const auto written = ::write(descriptors[1], value.data(), value.size());
  ::close(descriptors[1]);
  if (written != static_cast<ssize_t>(value.size())) {
    ::close(descriptors[0]);
    throw std::runtime_error("test owner input");
  }
  if (::setenv("XGC_PROCESS_OWNER_FD", std::to_string(descriptors[0]).c_str(), 1) != 0) {
    ::close(descriptors[0]);
    throw std::runtime_error("test owner environment");
  }
}

// What the server did to its robots and to the ROS binding, in order.
struct World {
  std::mutex mutex;
  std::condition_variable stop_changed;
  std::condition_variable retire_changed;
  std::atomic<std::uint64_t> command_pumps{0};
  std::atomic<std::uint64_t> native_cancel_attempts{0};
  std::atomic<int> control_silence_ms{3000};
  std::atomic<bool> hold_cancelled_native{false};
  std::atomic<bool> fail_publication_flush{false};
  std::atomic<bool> fail_native_retire{false};
  std::vector<std::string> log;
  struct Pending {
    operation::OperationRequest request;
    CommandCompletion complete;
    bool cancelled = false;
  };
  std::map<std::uint64_t, Pending> pending;
  std::uint64_t next_call = 0;
  bool complete_slow_motion = false;
  std::set<std::string> stopped;
  std::set<std::string> held_stops;
  std::set<std::string> entered_stops;
  std::set<std::string> failed_stops;
  std::set<std::string> held_retires;
  std::set<std::string> entered_retires;
  std::set<std::string> held_telemetry;
  std::map<std::string, int> telemetry_samples;
  std::map<std::string, int> periodic_ticks;
  std::map<std::string, int> health_reads;
  // CoreSilent() calls per robot and the longest silence reported.
  std::map<std::string, int> silent_calls;
  std::chrono::milliseconds longest_silence{0};

  int silentCalls(const std::string &robot) {
    std::lock_guard<std::mutex> lock(mutex);
    return silent_calls[robot];
  }

  void record(const std::string &line) {
    std::lock_guard<std::mutex> lock(mutex);
    log.push_back(line);
  }
  void holdStop(const std::string &robot) {
    std::lock_guard<std::mutex> lock(mutex);
    held_stops.insert(robot);
  }
  void releaseStop(const std::string &robot) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      held_stops.erase(robot);
    }
    stop_changed.notify_all();
  }
  bool waitStopEntered(const std::string &robot, milliseconds budget) {
    std::unique_lock<std::mutex> lock(mutex);
    return stop_changed.wait_for(lock, budget, [&] { return entered_stops.count(robot) != 0; });
  }
  void holdRetire(const std::string &robot) {
    std::lock_guard<std::mutex> lock(mutex);
    held_retires.insert(robot);
  }
  void releaseRetire(const std::string &robot) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      held_retires.erase(robot);
    }
    retire_changed.notify_all();
  }
  bool waitRetireEntered(const std::string &robot, milliseconds budget) {
    std::unique_lock<std::mutex> lock(mutex);
    return retire_changed.wait_for(lock, budget, [&] { return entered_retires.count(robot) != 0; });
  }
  std::vector<std::string> snapshot() {
    std::lock_guard<std::mutex> lock(mutex);
    return log;
  }
  std::size_t position(const std::string &line) {
    const auto lines = snapshot();
    const auto found = std::find(lines.begin(), lines.end(), line);
    return found == lines.end() ? lines.size() : found - lines.begin();
  }
  bool contains(const std::string &line) { return position(line) < snapshot().size(); }
  bool telemetryHeld(const std::string &robot, const std::string &channel) {
    std::lock_guard<std::mutex> lock(mutex);
    return held_telemetry.count(robot + "/" + channel) != 0;
  }
  void holdTelemetry(const std::string &robot, const std::string &channel) {
    std::lock_guard<std::mutex> lock(mutex);
    held_telemetry.insert(robot + "/" + channel);
  }
  void releaseTelemetry(const std::string &robot, const std::string &channel) {
    std::lock_guard<std::mutex> lock(mutex);
    held_telemetry.erase(robot + "/" + channel);
  }
  void recordTelemetry(const std::string &robot, const std::string &channel) {
    std::lock_guard<std::mutex> lock(mutex);
    ++telemetry_samples[robot + "/" + channel];
  }
  int telemetryCount(const std::string &robot, const std::string &channel) {
    std::lock_guard<std::mutex> lock(mutex);
    return telemetry_samples[robot + "/" + channel];
  }
  int telemetryCountForPrefix(const std::string &prefix) {
    std::lock_guard<std::mutex> lock(mutex);
    int total = 0;
    for (const auto &sample : telemetry_samples)
      if (sample.first.compare(0, prefix.size(), prefix) == 0)
        total += sample.second;
    return total;
  }
  void recordPeriodic(const std::string &robot) {
    std::lock_guard<std::mutex> lock(mutex);
    ++periodic_ticks[robot];
  }
  int periodicCountForPrefix(const std::string &prefix) {
    std::lock_guard<std::mutex> lock(mutex);
    int total = 0;
    for (const auto &sample : periodic_ticks)
      if (sample.first.compare(0, prefix.size(), prefix) == 0)
        total += sample.second;
    return total;
  }
  int healthReads(const std::string &robot) {
    std::lock_guard<std::mutex> lock(mutex);
    return health_reads[robot];
  }
};

operation::OperationEvent event(const operation::OperationRequest &request,
                                operation::OperationPhase phase) {
  operation::OperationEvent result;
  result.set_work_id(request.context().work_id());
  result.set_phase(phase);
  return result;
}

class FakeIo final : public CommandIo {
public:
  explicit FakeIo(World &world, bool real_wait = false) : world_(world) {
    if (real_wait)
      real_io_ = std::make_unique<AsyncRosServices>();
  }
  void Cancel(std::uint64_t id) override {
    ++world_.native_cancel_attempts;
    std::lock_guard<std::mutex> lock(world_.mutex);
    const auto found = world_.pending.find(id);
    if (found != world_.pending.end())
      found->second.cancelled = true;
  }
  void Pump(std::chrono::milliseconds wait) override {
    ++world_.command_pumps;
    std::vector<std::pair<CommandCompletion, operation::OperationEvent>> ready;
    {
      std::lock_guard<std::mutex> lock(world_.mutex);
      for (auto i = world_.pending.begin(); i != world_.pending.end();) {
        if (i->second.cancelled && !world_.hold_cancelled_native.load()) {
          ready.emplace_back(std::move(i->second.complete),
                             CommandError(i->second.request,
                                          operation::ERROR_CLASS_CANCELLED,
                                          "cancelled", "fake call cancelled"));
          world_.log.push_back("cancelled:" + i->second.request.context().work_id());
          i = world_.pending.erase(i);
        } else if (world_.complete_slow_motion && i->second.request.context().endpoint_id() == "slow-motion") {
          ready.emplace_back(std::move(i->second.complete),
                             event(i->second.request, operation::OPERATION_PHASE_SUCCEEDED));
          i = world_.pending.erase(i);
        } else {
          ++i;
        }
      }
    }
    for (auto &entry : ready)
      entry.first(std::move(entry.second));
    if (real_io_) {
      if (wait == milliseconds(100))
        world_.record("idle-pump");
      if (wait == milliseconds(0))
        world_.record("fence-pump");
      real_io_->Pump(wait);
    } else {
      std::this_thread::sleep_for(std::min(wait, milliseconds(2)));
    }
  }

private:
  World &world_;
  std::unique_ptr<AsyncRosServices> real_io_;
};

class FakeBinding final : public RosBinding {
public:
  explicit FakeBinding(World &world) : world_(world) {}
  // ---- script
  std::atomic<long> master_pid{4242};
  std::atomic<bool> master_run_known{true};
  std::atomic<bool> master_run_changed{false};
  std::atomic<bool> master_answers{true};
  std::atomic<bool> master_calls_fail{false}; // BoundMasterCalls reports failure
  std::atomic<bool> running{true};
  std::atomic<bool> hold_connect{false};
  bool real_command_wait = false;
  void releaseConnect() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      hold_connect = false;
    }
    changed_.notify_all();
  }
  ServerOptions initialized;
  std::string initialized_name;

  // ---- RosBinding
  void Init(const ServerOptions &options, const std::string &name) override {
    initialized = options;
    initialized_name = name;
    world_.record("init");
  }
  void Connect() override {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait(lock, [this] { return !hold_connect.load(); });
    world_.record("connect");
  }
  bool Running() const override { return running; }
  bool ProbeMaster(MasterIdentity *identity) override {
    if (!master_answers)
      return false;
    identity->uri = "http://devbox:11311/";
    identity->pid = master_pid;
    identity->run_id = MasterRunId();
    return true;
  }
  std::string MasterRunId() const override {
    return !master_run_known ? "" : master_run_changed
                                                   ? "master-run-2"
                                                   : "master-run-1";
  }
  void Shutdown() override { world_.record("ros-shutdown"); }
  std::size_t Lanes() const override { return 2; }
  void PumpLane(std::size_t, std::chrono::milliseconds wait) override {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait_for(lock, wait, [this] { return lanes_disabled_; });
  }
  void DisableLanes() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      lanes_disabled_ = true;
    }
    changed_.notify_all();
  }
  std::unique_ptr<RosNode> CreateNode(std::size_t) override {
    return std::unique_ptr<RosNode>(new RosNode());
  }
  std::unique_ptr<CommandIo> CreateCommandIo() override {
    return std::unique_ptr<CommandIo>(new FakeIo(world_, real_command_wait));
  }
  std::unique_ptr<MasterBudget>
  BoundMasterCalls(
      std::optional<std::chrono::milliseconds> budget) override {
    if (!budget)
      world_.record("master-tracking");
    struct Budget final : MasterBudget {
      explicit Budget(bool failure) : failure_(failure) {}
      bool failed() const override { return failure_; }
      bool failure_;
    };
    return std::unique_ptr<MasterBudget>(new Budget(master_calls_fail));
  }
  bool FlushPublications(std::chrono::milliseconds) override {
    world_.record(world_.fail_publication_flush ? "flush-timeout" : "flush");
    return !world_.fail_publication_flush;
  }

private:
  World &world_;
  std::mutex mutex_;
  std::condition_variable changed_;
  bool lanes_disabled_ = false;
};

// A robot whose commands are selected by the endpoint id of the request:
//   "instant"   succeeds at once
//   "slow"      stays in flight until the test finishes it
//   "uncertain" ends with an uncertain outcome
//   "release"   succeeds at once and counts as a safety operation
class FakeRobot final : public NativeRobot {
public:
  FakeRobot(World &world, RobotConfig config, TelemetryEmitter emit)
      : world_(world), config_(std::move(config)), emit_(std::move(emit)) {}
  ~FakeRobot() override { world_.record("destroy:" + config_.robot_id); }
  void Stop() override {
    std::unique_lock<std::mutex> lock(world_.mutex);
    world_.entered_stops.insert(config_.robot_id);
    world_.stop_changed.notify_all();
    world_.stop_changed.wait(lock, [&] { return !world_.held_stops.count(config_.robot_id); });
    if (world_.failed_stops.count(config_.robot_id)) {
      world_.log.push_back("stop-failed:" + config_.robot_id);
      throw std::runtime_error("controlled native Stop failure");
    }
    world_.log.push_back("stop:" + config_.robot_id);
    world_.stopped.insert(config_.robot_id);
  }
  void Periodic() override {
    world_.recordPeriodic(config_.robot_id);
    ++ticks_;
    if (ticks_ % 5 == 0 && std::getenv("XGC_T4_NATIVE_STATIC_STATUS") == nullptr) publish();
  }
  void SetStatusDemand(const std::set<std::string> &channels) override {
    std::lock_guard<std::mutex> lock(demand_mutex_);
    std::set<std::string> added;
    for (const auto &channel : channels)
      if (!demand_.count(channel))
        added.insert(channel);
    demand_ = channels;
    publishLocked(added);
  }
  std::optional<NativeHealth> DescribeHealth() override {
    std::lock_guard<std::mutex> lock(world_.mutex);
    ++world_.health_reads[config_.robot_id];
    return NativeHealth{true, "healthy"};
  }
  bool CoreSilent(std::chrono::milliseconds silent_for) override {
    std::lock_guard<std::mutex> lock(world_.mutex);
    ++world_.silent_calls[config_.robot_id];
    world_.longest_silence = std::max(world_.longest_silence, silent_for);
    return false;
  }
  bool RequiresControl(const operation::OperationRequest &request) const override {
    return request.context().endpoint_id() == "motion" || request.context().endpoint_id() == "slow-motion" ||
           request.context().endpoint_id() == "set-motion-intent";
  }
  bool IsControlRelease(const operation::OperationRequest &request) const override {
    return request.context().endpoint_id() == "release" || request.context().endpoint_id() == "release-motion-intent";
  }
  bool RetireControl(std::string *) override {
    std::unique_lock<std::mutex> lock(world_.mutex);
    world_.entered_retires.insert(config_.robot_id);
    world_.retire_changed.notify_all();
    world_.retire_changed.wait(lock, [&] { return !world_.held_retires.count(config_.robot_id); });
    if (world_.fail_native_retire) {
      world_.log.push_back("retire-failed:" + config_.robot_id);
      return false;
    }
    world_.log.push_back("retire-control:" + config_.robot_id);
    return true;
  }
  milliseconds ControlSilenceLimit() const override { return milliseconds(world_.control_silence_ms.load()); }
  bool IsSafetyOperation(const operation::OperationRequest &request) const override {
    return IsControlRelease(request);
  }
  std::uint64_t Execute(const operation::OperationRequest &request, CommandIo &,
                        CommandCompletion complete) override {
    const auto &kind = request.context().endpoint_id();
    world_.record("execute:" + config_.robot_id + ":" + kind);
    if (kind == "instant" || kind == "release" || kind == "motion" ||
        kind == "set-motion-intent" || kind == "release-motion-intent") {
      complete(event(request, operation::OPERATION_PHASE_SUCCEEDED));
      return 0;
    }
    if (kind == "uncertain") {
      complete(CommandError(request, operation::ERROR_CLASS_UNCERTAIN,
                            "native-result-unknown", "result unknown"));
      return 0;
    }
    std::lock_guard<std::mutex> lock(world_.mutex);
    const auto id = ++world_.next_call;
    world_.pending[id] = {request, std::move(complete), false};
    return id;
  }

private:
  void publish() {
    std::lock_guard<std::mutex> lock(demand_mutex_);
    publishLocked(demand_);
  }
  void publishLocked(const std::set<std::string> &channels) {
    for (const auto &channel : config_.channels) {
      if (!channel.enabled || !channels.count(channel.channel_id) ||
          world_.telemetryHeld(config_.robot_id, channel.channel_id))
        continue;
      xgc::robot::v1::RobotMessage message;
      message.set_robot_id(config_.robot_id);
      message.set_channel_id(channel.channel_id);
      message.mutable_message()->set_sequence(++sequence_);
      if (channel.channel_id == "command.velocity") {
        // Supply a valid controlled sample for the real Scout profile when
        // this test fixture is driven by the composed Core/Agent journey.
        auto *payload = message.mutable_message()->mutable_payload();
        auto *schema = payload->mutable_schema();
        schema->set_message_id(2002);
        schema->set_type_name("xgc.semantic.common.v1.VelocityEstimate");
        schema->set_schema_version(1);
        schema->set_schema_fingerprint(9039744259211632072ULL);
        payload->set_encoding(xgc::v1::PAYLOAD_ENCODING_JSON);
        payload->set_value("{\"linear\":{\"x\":1}}");
      }
      world_.recordTelemetry(config_.robot_id, channel.channel_id);
      emit_(std::move(message));
    }
  }
  World &world_;
  RobotConfig config_;
  TelemetryEmitter emit_;
  std::mutex demand_mutex_;
  std::set<std::string> demand_;
  std::uint64_t sequence_ = 0;
  unsigned ticks_ = 0;
};

RobotFactory factoryFor(World &world) {
  return [&world](RosNode &, const RobotConfig &config, TelemetryEmitter emit,
                  std::string *error) -> std::shared_ptr<NativeRobot> {
    if (config.profile_id == "rejected.profile") {
      *error = "profile does not match installed native mapping";
      return nullptr;
    }
    world.record("create:" + config.robot_id);
    return std::make_shared<FakeRobot>(world, config, std::move(emit));
  };
}

ServerTimings quickTimings() {
  ServerTimings timings;
  timings.master_probe_interval = milliseconds(20);
  timings.master_unreachable_after = milliseconds(300);
  timings.command_drain = milliseconds(400);
  timings.status_flush = milliseconds(150);
  timings.rpc_drain = milliseconds(2000);
  return timings;
}

std::string makePrivateDirectory() {
  char pattern[] = "/tmp/robot-server-test-XXXXXX";
  return mkdtemp(pattern);
}

void requireTlsFixture(bool valid) {
  if (!valid)
    throw std::runtime_error("could not create private TLS test fixture");
}

struct TestPem {
  std::string certificate, key;
};

TestPem testCertificate(const char *name, const TestPem *issuer = nullptr) {
  using Context = std::unique_ptr<EVP_PKEY_CTX, decltype(&EVP_PKEY_CTX_free)>;
  using Key = std::unique_ptr<EVP_PKEY, decltype(&EVP_PKEY_free)>;
  using Certificate = std::unique_ptr<X509, decltype(&X509_free)>;
  using Bio = std::unique_ptr<BIO, decltype(&BIO_free)>;
  using Extension = std::unique_ptr<X509_EXTENSION, decltype(&X509_EXTENSION_free)>;
  Context context(EVP_PKEY_CTX_new_id(issuer ? EVP_PKEY_ED25519 : EVP_PKEY_RSA, nullptr), EVP_PKEY_CTX_free);
  requireTlsFixture(context && EVP_PKEY_keygen_init(context.get()) == 1);
  if (!issuer)
    requireTlsFixture(EVP_PKEY_CTX_set_rsa_keygen_bits(context.get(), 2048) == 1);
  EVP_PKEY *generated = nullptr;
  requireTlsFixture(EVP_PKEY_keygen(context.get(), &generated) == 1);
  Key key(generated, EVP_PKEY_free), issuer_key(nullptr, EVP_PKEY_free);
  Certificate cert(X509_new(), X509_free), parent(nullptr, X509_free);
  requireTlsFixture(cert && X509_set_version(cert.get(), 2) == 1);
  static long serial = 1;
  requireTlsFixture(ASN1_INTEGER_set(X509_get_serialNumber(cert.get()), serial++) == 1);
  requireTlsFixture(X509_gmtime_adj(X509_get_notBefore(cert.get()), -60) != nullptr);
  requireTlsFixture(X509_gmtime_adj(X509_get_notAfter(cert.get()), 3600) != nullptr);
  requireTlsFixture(X509_set_pubkey(cert.get(), key.get()) == 1);
  auto *subject = X509_get_subject_name(cert.get());
  requireTlsFixture(X509_NAME_add_entry_by_txt(subject, "CN", MBSTRING_ASC,
      reinterpret_cast<const unsigned char *>(name), -1, -1, 0) == 1);
  if (issuer) {
    Bio ca(BIO_new_mem_buf(issuer->certificate.data(), static_cast<int>(issuer->certificate.size())), BIO_free);
    Bio private_key(BIO_new_mem_buf(issuer->key.data(), static_cast<int>(issuer->key.size())), BIO_free);
    parent.reset(PEM_read_bio_X509(ca.get(), nullptr, nullptr, nullptr));
    issuer_key.reset(PEM_read_bio_PrivateKey(private_key.get(), nullptr, nullptr, nullptr));
    requireTlsFixture(parent && issuer_key);
  }
  requireTlsFixture(X509_set_issuer_name(cert.get(), parent ? X509_get_subject_name(parent.get()) : subject) == 1);
  const auto extension = [&](int id, const char *value) {
    Extension item(X509V3_EXT_conf_nid(nullptr, nullptr, id, const_cast<char *>(value)), X509_EXTENSION_free);
    requireTlsFixture(item && X509_add_ext(cert.get(), item.get(), -1) == 1);
  };
  extension(NID_basic_constraints, issuer ? "critical,CA:FALSE" : "critical,CA:TRUE");
  extension(NID_key_usage, issuer ? "critical,digitalSignature" : "critical,keyCertSign,cRLSign");
  if (issuer) {
    extension(NID_ext_key_usage, "serverAuth,clientAuth");
    extension(NID_subject_alt_name, "DNS:localhost,IP:127.0.0.1");
  }
  requireTlsFixture(X509_sign(cert.get(), issuer_key ? issuer_key.get() : key.get(), EVP_sha256()) > 0);
  Bio certificate(BIO_new(BIO_s_mem()), BIO_free), private_key(BIO_new(BIO_s_mem()), BIO_free);
  requireTlsFixture(certificate && private_key);
  requireTlsFixture(PEM_write_bio_X509(certificate.get(), cert.get()) == 1);
  requireTlsFixture(PEM_write_bio_PrivateKey(private_key.get(), key.get(), nullptr, nullptr, 0, nullptr, nullptr) == 1);
  const auto bytes = [](BIO *bio) {
    char *data = nullptr;
    const auto size = BIO_get_mem_data(bio, &data);
    requireTlsFixture(size > 0);
    return std::string(data, static_cast<std::size_t>(size));
  };
  return {bytes(certificate.get()), bytes(private_key.get())};
}

struct TlsDirectory {
  std::string path = makePrivateDirectory();
  ~TlsDirectory() {
    if (DIR *directory = opendir(path.c_str())) {
      while (auto *entry = readdir(directory)) {
        const std::string name(entry->d_name);
        if (name != "." && name != "..")
          unlink((path + "/" + name).c_str());
      }
      closedir(directory);
    }
    rmdir(path.c_str());
  }
};

struct TlsFixture {
  TlsDirectory directory;
  TestPem ca = testCertificate("adapter test CA");
  TestPem server = testCertificate("localhost", &ca);
  TestPem client = testCertificate("Core test client", &ca);
  int port = 0;
  unsigned files = 0;
  TlsFixture() {
    const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    requireTlsFixture(fd >= 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    requireTlsFixture(bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0);
    socklen_t size = sizeof(address);
    requireTlsFixture(getsockname(fd, reinterpret_cast<sockaddr *>(&address), &size) == 0);
    port = ntohs(address.sin_port);
    requireTlsFixture(close(fd) == 0);
  }
  std::string file(const std::string &value) {
    const auto path = directory.path + "/" + std::to_string(files++);
    const int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    requireTlsFixture(fd >= 0);
    std::size_t offset = 0;
    while (offset != value.size()) {
      const auto count = write(fd, value.data() + offset, value.size() - offset);
      requireTlsFixture(count > 0);
      offset += static_cast<std::size_t>(count);
    }
    requireTlsFixture(close(fd) == 0);
    return path;
  }
  std::string bootstrap(const std::string &target = "local",
                        const std::string &service = "xgc.robot.v1.RobotAdapterServerService",
                        const std::string &version = "v2") {
    const auto certificate = file(server.certificate), key = file(server.key);
    const auto trust = file(ca.certificate), token = file("test-gateway-token");
    return file(nlohmann::json{
        {"schema_version", 1},
        {"binding", {{"schema_version", 1}, {"target_id", target}, {"service", service},
                     {"api_version", version}, {"profile", "grpc.v1"},
                     {"endpoint", {{"kind", "tls"}, {"address", "127.0.0.1:" + std::to_string(port)}}},
                     {"runtime_grant", "owner.runtime"}, {"authentication", "mutual_tls"},
                     {"secret_handles", {{"tls_identity", "owner.identity"}, {"tls_trust", "owner.trust"},
                                          {"authorization", "owner.auth"}}},
                     {"storage_grants", nlohmann::json::array()}}},
        {"grants", {{"owner.identity", {{"kind", "tls_identity"}, {"cert_file", certificate}, {"key_file", key}}},
                    {"owner.trust", {{"kind", "tls_trust"}, {"ca_file", trust}}},
                    {"owner.auth", {{"kind", "bearer"}, {"token_file", token}}}}}}
                    .dump());
  }
  std::shared_ptr<grpc::Channel> channel(const std::string &address) const {
    grpc::SslCredentialsOptions credentials;
    credentials.pem_root_certs = ca.certificate;
    credentials.pem_cert_chain = client.certificate;
    credentials.pem_private_key = client.key;
    grpc::ChannelArguments arguments;
    arguments.SetInt(GRPC_ARG_ENABLE_RETRIES, 0);
    return grpc::CreateCustomChannel(address, grpc::SslCredentials(credentials), arguments);
  }
};

wire::MemberIdentity identity(const std::string &robot, std::uint64_t epoch = 1,
                              const std::string &run = "run-test") {
  wire::MemberIdentity result;
  result.set_target_id("local");
  result.set_run_id(run);
  result.set_robot_id(robot);
  result.set_connection_epoch(epoch);
  return result;
}

wire::MemberRegistration registration(const std::string &robot,
                                      const std::string &robot_namespace,
                                      const std::string &profile = "fake.profile",
                                      std::uint64_t epoch = 1,
                                      const std::string &run = "run-test") {
  wire::MemberRegistration result;
  *result.mutable_identity() = identity(robot, epoch, run);
  auto *configuration = result.mutable_configuration();
  configuration->set_robot_id(robot);
  configuration->set_profile_id(profile);
  (*configuration->mutable_parameters())["namespace"] = robot_namespace;
  auto *channel = configuration->add_channels();
  channel->set_channel_id("state.test");
  channel->set_enabled(true);
  return result;
}

wire::ExecuteRequest command(const std::string &robot, const std::string &kind,
                             const std::string &work, std::uint64_t epoch = 1) {
  wire::ExecuteRequest request;
  *request.mutable_identity() = identity(robot, epoch);
  auto *context = request.mutable_operation()->mutable_context();
  context->set_work_id(work);
  context->set_endpoint_id(kind);
  context->mutable_deadline()->set_deadline_unix_nanos(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          (std::chrono::system_clock::now() + std::chrono::seconds(60))
              .time_since_epoch())
          .count());
  return request;
}

// One RPC outcome as a caller sees it.
template <class Response> struct Reply {
  grpc::Status status;
  xgc2::xrpc::Delivery delivery = xgc2::xrpc::Delivery::NotSent;
  Response response;
};

class Client {
public:
  explicit Client(const std::string &socket)
      : Client(xgc2::xrpc::make_grpc_unix_channel(socket)) {}
  explicit Client(std::shared_ptr<grpc::Channel> channel,
                  std::string authorization = {}, std::string instance = {})
      : stub_(wire::RobotAdapterServerService::NewStub(std::move(channel))),
        instance_(std::move(instance)), authorization_(std::move(authorization)) {}

  Reply<wire::DescribeResponse> describe(std::uint32_t wait_ms = 0,
                                         milliseconds deadline = milliseconds(10000)) {
    Reply<wire::DescribeResponse> reply;
    grpc::ClientContext context;
    authorize(context);
    GrpcClientCall call(context, "", GrpcClock::now() + deadline, {}, "", true);
    wire::DescribeRequest request;
    request.set_wait_ready_ms(wait_ms);
    reply.status = call.invoke(
        [&] { return stub_->Describe(&context, request, &reply.response); });
    reply.delivery = call.delivery();
    if (reply.status.ok())
      instance_ = call.response_instance_id();
    return reply;
  }
  template <class Response, class Request, class Method>
  Reply<Response> call(const Request &request, Method method,
                       milliseconds deadline = milliseconds(10000)) {
    if (instance_.empty())
      describe();
    Reply<Response> reply;
    grpc::ClientContext context;
    authorize(context);
    GrpcClientCall call(context, instance_, GrpcClock::now() + deadline);
    reply.status = call.invoke([&] {
      return (stub_.get()->*method)(&context, request, &reply.response);
    });
    reply.delivery = call.delivery();
    return reply;
  }
  Reply<wire::ApplyMembersResponse> apply(const wire::ApplyMembersRequest &request) {
    return call<wire::ApplyMembersResponse>(
        request, &wire::RobotAdapterServerService::Stub::ApplyMembers);
  }
  Reply<wire::RemoveMembersResponse> remove(const wire::RemoveMembersRequest &request) {
    return call<wire::RemoveMembersResponse>(
        request, &wire::RobotAdapterServerService::Stub::RemoveMembers);
  }
  Reply<wire::ExecuteResponse> execute(const wire::ExecuteRequest &request,
                                       milliseconds deadline = milliseconds(10000)) {
    return call<wire::ExecuteResponse>(
        request, &wire::RobotAdapterServerService::Stub::Execute, deadline);
  }
  Reply<wire::HealthResponse> health() {
    return call<wire::HealthResponse>(
        wire::HealthRequest(), &wire::RobotAdapterServerService::Stub::Health);
  }
  Reply<wire::ShutdownResponse> shutdown(milliseconds deadline = milliseconds(10000)) {
    return call<wire::ShutdownResponse>(
        wire::ShutdownRequest(), &wire::RobotAdapterServerService::Stub::Shutdown,
        deadline);
  }

  // A status stream read on a thread; frames are kept for the test to inspect.
  class Watch {
  public:
    Watch(Client &client, milliseconds duration,
          wire::SubscribeStatusRequest request) {
      if (client.instance_.empty())
        client.describe();
      client.authorize(context_);
      call_ = std::make_unique<GrpcClientCall>(
          context_, client.instance_,
          xgc2::xrpc::grpc_stream_deadline(xgc2::xrpc::GrpcLimits{},
                                           GrpcClock::now() + duration));
      if (!call_->mark_dispatched().ok())
        return;
      stream_ = client.stub_->SubscribeStatus(&context_, request);
      if (!call_->receive_initial_metadata(*stream_).ok())
        return;
      reader_ = std::thread([this] {
        wire::SubscribeStatusResponse frame;
        while (stream_->Read(&frame)) {
          std::lock_guard<std::mutex> lock(mutex_);
          frames_.push_back(frame);
          changed_.notify_all();
        }
        const auto status = stream_->Finish();
        std::lock_guard<std::mutex> lock(mutex_);
        finished_ = true;
        status_ = status;
        changed_.notify_all();
      });
    }
    ~Watch() {
      if (reader_.joinable()) {
        context_.TryCancel();
        reader_.join();
      }
    }
    // Waits until a frame satisfies the predicate.
    template <class Predicate>
    bool wait(Predicate predicate, milliseconds budget = milliseconds(5000)) {
      std::unique_lock<std::mutex> lock(mutex_);
      return changed_.wait_for(lock, budget, [&] {
        for (const auto &frame : frames_)
          if (predicate(frame))
            return true;
        return false;
      });
    }
    bool waitFinished(milliseconds budget = milliseconds(5000)) {
      std::unique_lock<std::mutex> lock(mutex_);
      return changed_.wait_for(lock, budget, [&] { return finished_; });
    }
    grpc::Status status() {
      std::lock_guard<std::mutex> lock(mutex_);
      return status_;
    }
    static wire::SubscribeStatusRequest broadSelection() {
      wire::SubscribeStatusRequest request;
      request.set_all(true);
      return request;
    }
    std::vector<wire::SubscribeStatusResponse> frames() {
      std::lock_guard<std::mutex> lock(mutex_);
      return frames_;
    }
    void cancel() { context_.TryCancel(); }

  private:
    grpc::ClientContext context_;
    std::unique_ptr<GrpcClientCall> call_;
    std::unique_ptr<grpc::ClientReader<wire::SubscribeStatusResponse>> stream_;
    std::thread reader_;
    std::mutex mutex_;
    std::condition_variable changed_;
    std::vector<wire::SubscribeStatusResponse> frames_;
    bool finished_ = false;
    grpc::Status status_;
  };

private:
  void authorize(grpc::ClientContext &context) const {
    if (!authorization_.empty())
      context.AddMetadata("authorization", authorization_);
  }
  std::unique_ptr<wire::RobotAdapterServerService::Stub> stub_;
  std::string instance_;
  std::string authorization_;
};

// One server on its own thread.
class Fixture : public ::testing::Test {
protected:
  void SetUp() override {
    directory_ = makePrivateDirectory();
    binding_ = std::make_unique<FakeBinding>(world_);
  }
  void TearDown() override {
    binding_->releaseConnect();
    if (thread_.joinable()) {
      server_->RequestStop();
      thread_.join();
    }
    client_.reset();
    server_.reset();
    unlink(socket().c_str());
    rmdir(directory_.c_str());
  }
  std::string socket() const { return directory_ + "/server.sock"; }
  ServerOptions options() const {
    ServerOptions result;
    result.socket_path = socket();
    result.run_id = "run-test";
    result.node_name = "xgc_fake_run-test";
    result.provider = "fake-provider";
    result.target_id = "local";
    result.ros_master_uri = "http://127.0.0.1:11311";
    result.ros_ip = "127.0.0.1";
    return result;
  }
  void start(ServerTimings timings = quickTimings(),
             const std::string &network_bootstrap = {}) {
    auto configured = options();
    configured.network_bootstrap = network_bootstrap;
    prepareOwnerInput(kTestProcessOwner);
    server_ = std::make_unique<RobotServer>(std::move(configured), *binding_,
                                            factoryFor(world_), timings);
    finished_ = false;
    thread_ = std::thread([this] {
      exit_code_ = server_->Run();
      finished_ = true;
    });
    for (int attempt = 0;
         attempt < 500 && !finished_ && access(socket().c_str(), F_OK); ++attempt)
      std::this_thread::sleep_for(milliseconds(5));
    client_ = std::make_unique<Client>(socket());
  }
  void startReady() {
    start();
    const auto described = client_->describe(10000);
    ASSERT_TRUE(described.status.ok()) << described.status.error_message();
    ASSERT_TRUE(described.response.ready());
  }
  int join() {
    thread_.join();
    return exit_code_;
  }
  static nlohmann::json facts(const wire::DescribeResponse &response) {
    return nlohmann::json::parse(response.facts_json());
  }

  World world_;
  std::unique_ptr<FakeBinding> binding_;
  std::unique_ptr<RobotServer> server_;
  std::unique_ptr<Client> client_;
  std::thread thread_;
  std::string directory_;
  int exit_code_ = -1;
  std::atomic<bool> finished_{false};
};

using ServerLifecycle = Fixture;
using ServerNetwork = Fixture;
using ServerController = Fixture;

TEST_F(ServerController, PrivateStartupInputIsMandatoryAndExact) {
  ::unsetenv("XGC_PROCESS_OWNER_FD");
  EXPECT_THROW({ RobotServer server(options(), *binding_, factoryFor(world_)); }, std::invalid_argument);
  for (const std::size_t size : {std::size_t(31), std::size_t(33)}) {
    prepareOwnerInput(std::string(size, 'p'));
    EXPECT_THROW({ RobotServer server(options(), *binding_, factoryFor(world_)); }, std::invalid_argument);
    EXPECT_EQ(nullptr, std::getenv("XGC_PROCESS_OWNER_FD"));
  }
}

TEST_F(ServerController, InstallationIsExactAndRetiredGenerationsCannotReopen) {
  startReady();
  const auto described = client_->describe();
  ASSERT_TRUE(described.status.ok());
  wire::InstallControllerRequest request;
  request.set_process_owner(kTestProcessOwner);
  request.set_generation(1);
  request.set_open_id(std::string(16, 'o'));
  request.set_controller_grant(std::string(32, 'g'));
  request.mutable_binding()->set_provider_instance_id(described.response.service_ref().instance_id());
  request.mutable_binding()->set_core_id("core");
  request.mutable_binding()->set_core_boot_id("boot");
  auto install = [&](const wire::InstallControllerRequest &value) {
    return client_->call<wire::InstallControllerResponse>(value, &wire::RobotAdapterServerService::Stub::InstallController);
  };
  auto wrong = request;
  wrong.set_process_owner(std::string(32, 'x'));
  EXPECT_EQ(grpc::StatusCode::PERMISSION_DENIED, install(wrong).status.error_code());
  wrong = request;
  wrong.mutable_binding()->set_provider_instance_id("foreign");
  EXPECT_EQ(grpc::StatusCode::INVALID_ARGUMENT, install(wrong).status.error_code());
  auto installed = install(request);
  ASSERT_TRUE(installed.status.ok()) << installed.status.error_message();
  EXPECT_EQ(1u, installed.response.generation());
  EXPECT_EQ(request.open_id(), installed.response.open_id());
  ASSERT_TRUE(install(request).status.ok());
  wrong = request;
  wrong.set_controller_grant(std::string(32, 'h'));
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION, install(wrong).status.error_code());
  wrong = request;
  wrong.set_generation(2);
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION, install(wrong).status.error_code());
  wire::RetireControllerRequest retire;
  retire.set_process_owner(kTestProcessOwner);
  retire.set_generation(1);
  retire.set_open_id(request.open_id());
  auto retireCall = [&](const wire::RetireControllerRequest &value) {
    return client_->call<wire::RetireControllerResponse>(value, &wire::RobotAdapterServerService::Stub::RetireController);
  };
  auto retired = retireCall(retire);
  ASSERT_TRUE(retired.status.ok()) << retired.status.error_message();
  EXPECT_TRUE(retired.response.joined());
  EXPECT_TRUE(retireCall(retire).response.joined());
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION, install(request).status.error_code());
  wrong = request;
  wrong.set_generation(2);
  wrong.set_open_id(std::string(16, 'n'));
  ASSERT_TRUE(install(wrong).status.ok());
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION, retireCall(retire).status.error_code());
}

TEST_F(ServerController, TlsServiceCredentialAndOwnerBytesCannotInstall) {
  TlsFixture network;
  start(quickTimings(), network.bootstrap());
  const auto described = client_->describe(10000);
  ASSERT_TRUE(described.status.ok());
  const auto address = "127.0.0.1:" + std::to_string(network.port);
  Client remote(network.channel(address), "Bearer test-gateway-token", described.response.service_ref().instance_id());
  wire::InstallControllerRequest request;
  request.set_process_owner(kTestProcessOwner);
  request.set_generation(1);
  request.set_open_id(std::string(16, 'o'));
  request.set_controller_grant(std::string(32, 'g'));
  request.mutable_binding()->set_provider_instance_id(described.response.service_ref().instance_id());
  request.mutable_binding()->set_core_id("core");
  request.mutable_binding()->set_core_boot_id("boot");
  const auto denied = remote.call<wire::InstallControllerResponse>(request, &wire::RobotAdapterServerService::Stub::InstallController);
  EXPECT_EQ(grpc::StatusCode::PERMISSION_DENIED, denied.status.error_code());
  const auto local = client_->call<wire::InstallControllerResponse>(request, &wire::RobotAdapterServerService::Stub::InstallController);
  ASSERT_TRUE(local.status.ok()) << local.status.error_message();
}

TEST_F(ServerController, RequiredMemberControlIsIndependentOfOptionalViewsAndJoinsRetirement) {
  startReady();
  world_.control_silence_ms = 600;
  const auto described = client_->describe();
  ASSERT_TRUE(described.status.ok());
  wire::ApplyMembersRequest add;
  *add.add_members() = registration("r1", "/fleet/r1");
  *add.add_members() = registration("r2", "/fleet/r2");
  ASSERT_TRUE(client_->apply(add).status.ok());
  const std::string grant(32, 'g');
  wire::InstallControllerRequest install;
  install.set_process_owner(kTestProcessOwner);
  install.set_generation(1);
  install.set_open_id(std::string(16, 'i'));
  install.set_controller_grant(grant);
  install.mutable_binding()->set_provider_instance_id(described.response.service_ref().instance_id());
  install.mutable_binding()->set_core_id("core");
  install.mutable_binding()->set_core_boot_id("boot");
  ASSERT_TRUE(client_->call<wire::InstallControllerResponse>(install,
      &wire::RobotAdapterServerService::Stub::InstallController).status.ok());
  wire::AcquireControlRequest acquire;
  acquire.set_controller_grant(grant);
  *acquire.mutable_identity() = identity("r1");
  acquire.set_generation(1);
  auto acquireCall = [&](const wire::AcquireControlRequest &value) {
    return client_->call<wire::AcquireControlResponse>(value, &wire::RobotAdapterServerService::Stub::AcquireControl);
  };
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION, acquireCall(acquire).status.error_code());
  auto unauthorized = command("r1", "motion", "without-control");
  EXPECT_EQ("control-unavailable", client_->execute(unauthorized).response.event().error().code());
  EXPECT_FALSE(world_.contains("execute:r1:motion"));

  wire::SubscribeStatusRequest control;
  control.set_control(true);
  control.mutable_owner()->set_controller_grant(grant);
  control.mutable_owner()->set_generation(1);
  control.mutable_owner()->set_open_id(std::string(16, 'a'));
  Client::Watch required(*client_, milliseconds(20000), control);
  ASSERT_TRUE(required.wait([&](const wire::SubscribeStatusResponse &frame) {
    return frame.generation() == 1 && frame.open_id() == control.owner().open_id() && !frame.members_size();
  }));
  const auto admitted = acquireCall(acquire);
  ASSERT_TRUE(admitted.status.ok());
  EXPECT_EQ(200u, admitted.response.heartbeat_interval_ms());
  ASSERT_TRUE(acquireCall(acquire).status.ok());
  auto motion = command("r1", "motion", "owned-motion");
  motion.set_controller_grant(grant);
  motion.set_control_generation(1);
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, client_->execute(motion).response.event().phase());
  auto foreign = command("r2", "motion", "foreign-member");
  foreign.set_controller_grant(grant);
  foreign.set_control_generation(1);
  EXPECT_EQ("control-unavailable", client_->execute(foreign).response.event().error().code());
  EXPECT_FALSE(world_.contains("execute:r2:motion"));
  auto other_control = acquire;
  *other_control.mutable_identity() = identity("r2");
  ASSERT_TRUE(acquireCall(other_control).status.ok());
  foreign.mutable_operation()->mutable_context()->set_work_id("owned-other-motion");
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, client_->execute(foreign).response.event().phase());

  wire::SubscribeStatusRequest selection;
  auto *item = selection.mutable_selected()->add_members();
  *item->mutable_identity() = identity("r2");
  item->add_channel_ids("state.test");
  std::vector<std::unique_ptr<Client::Watch>> views;
  for (int index = 0; index < 3; ++index) {
    views.emplace_back(new Client::Watch(*client_, milliseconds(20000), selection));
    ASSERT_TRUE(views.back()->wait([](const wire::SubscribeStatusResponse &frame) {
      return frame.members_size() && frame.members(0).messages_size();
    }));
  }
  EXPECT_EQ(4u, server_->SnapshotStatusWork().subscribers);
  std::this_thread::sleep_for(milliseconds(350));
  wire::RenewControlRequest renew;
  renew.set_controller_grant(grant);
  renew.set_status_generation(1);
  renew.set_status_open_id(control.owner().open_id());
  auto *renewed_member = renew.add_members();
  *renewed_member->mutable_identity() = acquire.identity();
  renewed_member->set_generation(1);
  auto *stale_member = renew.add_members();
  *stale_member->mutable_identity() = other_control.identity();
  stale_member->set_generation(9);
  auto renewCall = [&](const wire::RenewControlRequest &value) {
    return client_->call<wire::RenewControlResponse>(value, &wire::RobotAdapterServerService::Stub::RenewControl);
  };
  const auto renewed = renewCall(renew);
  ASSERT_TRUE(renewed.status.ok());
  ASSERT_EQ(2, renewed.response.members_size());
  EXPECT_TRUE(renewed.response.members(0).renewed());
  EXPECT_FALSE(renewed.response.members(1).renewed());
  EXPECT_EQ(grpc::StatusCode::RESOURCE_EXHAUSTED, renewCall(renew).status.error_code());
  EXPECT_EQ(1u, server_->SnapshotStatusWork().control_renewals);
  EXPECT_EQ(1u, server_->SnapshotStatusWork().control_renewal_rejections);
  EXPECT_GT(server_->SnapshotStatusWork().control_renewal_request_bytes, 0u);
  EXPECT_GT(server_->SnapshotStatusWork().control_renewal_response_bytes, 0u);
  const auto r1_before = world_.silentCalls("r1");
  const auto r2_before = world_.silentCalls("r2");
  std::this_thread::sleep_for(milliseconds(400));
  EXPECT_EQ(r1_before, world_.silentCalls("r1"));
  EXPECT_GT(world_.silentCalls("r2"), r2_before);
  EXPECT_EQ(1u, required.frames().size());

  wire::RetireStatusRequest retire;
  *retire.mutable_owner() = control.owner();
  retire.set_control(true);
  auto retireCall = [&](const wire::RetireStatusRequest &value) {
    return client_->call<wire::RetireStatusResponse>(value, &wire::RobotAdapterServerService::Stub::RetireStatus);
  };
  const auto before_join = server_->SnapshotStatusWork().subscribers_joined;
  const auto joined = retireCall(retire);
  ASSERT_TRUE(joined.status.ok()) << joined.status.error_message();
  EXPECT_TRUE(joined.response.joined());
  EXPECT_GT(server_->SnapshotStatusWork().subscribers_joined, before_join);
  ASSERT_TRUE(required.waitFinished());
  EXPECT_TRUE(retireCall(retire).response.joined());
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION, acquireCall(acquire).status.error_code());
  const auto silent_before = world_.silentCalls("r1");
  std::this_thread::sleep_for(milliseconds(300));
  EXPECT_GT(world_.silentCalls("r1"), silent_before);

  wire::SubscribeStatusRequest renewal = control;
  renewal.mutable_owner()->set_predecessor_generation(1);
  renewal.mutable_owner()->set_predecessor_open_id(control.owner().open_id());
  renewal.mutable_owner()->set_generation(2);
  renewal.mutable_owner()->set_open_id(std::string(16, 'b'));
  Client::Watch successor(*client_, milliseconds(20000), renewal);
  ASSERT_TRUE(successor.wait([](const wire::SubscribeStatusResponse &frame) { return frame.generation() == 2; }));
  ASSERT_TRUE(acquireCall(acquire).status.ok());
  wire::ReleaseControlRequest release;
  release.set_controller_grant(grant);
  *release.mutable_identity() = acquire.identity();
  release.set_generation(1);
  auto releaseCall = [&](const wire::ReleaseControlRequest &value) {
    return client_->call<wire::ReleaseControlResponse>(value, &wire::RobotAdapterServerService::Stub::ReleaseControl);
  };
  const auto released = releaseCall(release);
  ASSERT_TRUE(released.status.ok()) << released.status.error_message();
  EXPECT_TRUE(released.response.joined());
  EXPECT_TRUE(world_.contains("retire-control:r1"));
  EXPECT_TRUE(releaseCall(release).response.joined());
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION, acquireCall(acquire).status.error_code());
  acquire.set_generation(2);
  ASSERT_TRUE(acquireCall(acquire).status.ok());
  wire::RetireControllerRequest end;
  end.set_process_owner(kTestProcessOwner);
  end.set_generation(1);
  end.set_open_id(install.open_id());
  const auto ended = client_->call<wire::RetireControllerResponse>(end,
      &wire::RobotAdapterServerService::Stub::RetireController);
  ASSERT_TRUE(ended.status.ok()) << ended.status.error_message();
  EXPECT_TRUE(ended.response.joined());
  EXPECT_TRUE(successor.waitFinished());
  EXPECT_EQ(3u, server_->SnapshotStatusWork().subscribers);
}

TEST_F(ServerController, UnknownOpenIsFencedBeforeLateAdmission) {
  startReady();
  const auto described = client_->describe();
  wire::InstallControllerRequest install;
  install.set_process_owner(kTestProcessOwner);
  install.set_generation(1);
  install.set_open_id(std::string(16, 'i'));
  install.set_controller_grant(std::string(32, 'g'));
  install.mutable_binding()->set_provider_instance_id(described.response.service_ref().instance_id());
  install.mutable_binding()->set_core_id("core");
  install.mutable_binding()->set_core_boot_id("boot");
  ASSERT_TRUE(client_->call<wire::InstallControllerResponse>(install,
      &wire::RobotAdapterServerService::Stub::InstallController).status.ok());
  wire::SubscribeStatusRequest open;
  open.set_control(true);
  open.mutable_owner()->set_controller_grant(install.controller_grant());
  open.mutable_owner()->set_generation(1);
  open.mutable_owner()->set_open_id(std::string(16, 'a'));
  wire::RetireStatusRequest retire;
  *retire.mutable_owner() = open.owner();
  retire.set_control(true);
  const auto fenced = client_->call<wire::RetireStatusResponse>(retire,
      &wire::RobotAdapterServerService::Stub::RetireStatus);
  ASSERT_TRUE(fenced.status.ok());
  EXPECT_TRUE(fenced.response.joined());
  Client::Watch stale(*client_, milliseconds(5000), open);
  ASSERT_TRUE(stale.waitFinished());
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION, stale.status().error_code());
  open.mutable_owner()->set_predecessor_generation(1);
  open.mutable_owner()->set_predecessor_open_id(std::string(16, 'a'));
  open.mutable_owner()->set_generation(2);
  open.mutable_owner()->set_open_id(std::string(16, 'b'));
  Client::Watch current(*client_, milliseconds(10000), open);
  ASSERT_TRUE(current.wait([](const wire::SubscribeStatusResponse &frame) { return frame.generation() == 2; }));
}

TEST_F(ServerController, RetireWaitsForAcceptedMotionAndNativeRelease) {
  startReady();
  const auto described = client_->describe();
  wire::ApplyMembersRequest add;
  *add.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(add).status.ok());
  wire::InstallControllerRequest install;
  install.set_process_owner(kTestProcessOwner);
  install.set_generation(1);
  install.set_open_id(std::string(16, 'i'));
  install.set_controller_grant(std::string(32, 'g'));
  install.mutable_binding()->set_provider_instance_id(described.response.service_ref().instance_id());
  install.mutable_binding()->set_core_id("core");
  install.mutable_binding()->set_core_boot_id("boot");
  ASSERT_TRUE(client_->call<wire::InstallControllerResponse>(install,
      &wire::RobotAdapterServerService::Stub::InstallController).status.ok());
  wire::SubscribeStatusRequest open;
  open.set_control(true);
  open.mutable_owner()->set_controller_grant(install.controller_grant());
  open.mutable_owner()->set_generation(1);
  open.mutable_owner()->set_open_id(std::string(16, 'a'));
  Client::Watch control(*client_, milliseconds(10000), open);
  ASSERT_TRUE(control.wait([](const wire::SubscribeStatusResponse &frame) { return frame.generation() == 1; }));
  wire::AcquireControlRequest acquire;
  acquire.set_controller_grant(install.controller_grant());
  *acquire.mutable_identity() = identity("r1");
  acquire.set_generation(1);
  ASSERT_TRUE(client_->call<wire::AcquireControlResponse>(acquire,
      &wire::RobotAdapterServerService::Stub::AcquireControl).status.ok());
  auto motion = command("r1", "slow-motion", "slow-motion-work");
  motion.set_controller_grant(install.controller_grant());
  motion.set_control_generation(1);
  auto executing = std::async(std::launch::async, [&] {
    Client actor(socket());
    return actor.execute(motion);
  });
  for (int attempt = 0; attempt < 200 && !world_.contains("execute:r1:slow-motion"); ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  ASSERT_TRUE(world_.contains("execute:r1:slow-motion"));
  wire::RetireControllerRequest retire;
  retire.set_process_owner(kTestProcessOwner);
  retire.set_generation(1);
  retire.set_open_id(install.open_id());
  auto ending = std::async(std::launch::async, [&] {
    Client owner(socket());
    return owner.call<wire::RetireControllerResponse>(retire,
        &wire::RobotAdapterServerService::Stub::RetireController);
  });
  EXPECT_EQ(std::future_status::timeout, ending.wait_for(milliseconds(150)));
  EXPECT_FALSE(world_.contains("retire-control:r1"));
  auto successor = install;
  successor.set_generation(2);
  successor.set_open_id(std::string(16, 'n'));
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION,
      client_->call<wire::InstallControllerResponse>(successor,
          &wire::RobotAdapterServerService::Stub::InstallController).status.error_code());
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.complete_slow_motion = true;
  }
  const auto command_result = executing.get();
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, command_result.response.event().phase());
  const auto retired = ending.get();
  ASSERT_TRUE(retired.status.ok()) << retired.status.error_message();
  EXPECT_TRUE(retired.response.joined());
  EXPECT_TRUE(world_.contains("retire-control:r1"));
  EXPECT_TRUE(control.waitFinished());
  EXPECT_TRUE(client_->call<wire::InstallControllerResponse>(successor,
      &wire::RobotAdapterServerService::Stub::InstallController).status.ok());
}


// A native publication failure cannot produce a joined controller receipt or
// let a successor reuse the provider. An exact retry may finish the release.
TEST_F(ServerController, FailedNativeReleaseRetainsControllerUntilExactRetry) {
  startReady();
  const auto described = client_->describe();
  wire::ApplyMembersRequest add;
  *add.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(add).status.ok());
  wire::InstallControllerRequest install;
  install.set_process_owner(kTestProcessOwner);
  install.set_generation(1);
  install.set_open_id(std::string(16, 'i'));
  install.set_controller_grant(std::string(32, 'g'));
  install.mutable_binding()->set_provider_instance_id(described.response.service_ref().instance_id());
  install.mutable_binding()->set_core_id("core");
  install.mutable_binding()->set_core_boot_id("boot");
  ASSERT_TRUE(client_->call<wire::InstallControllerResponse>(install,
      &wire::RobotAdapterServerService::Stub::InstallController).status.ok());
  wire::SubscribeStatusRequest open;
  open.set_control(true);
  open.mutable_owner()->set_controller_grant(install.controller_grant());
  open.mutable_owner()->set_generation(1);
  open.mutable_owner()->set_open_id(std::string(16, 'a'));
  Client::Watch control(*client_, milliseconds(10000), open);
  ASSERT_TRUE(control.wait([](const wire::SubscribeStatusResponse &frame) { return frame.generation() == 1; }));
  wire::AcquireControlRequest acquire;
  acquire.set_controller_grant(install.controller_grant());
  *acquire.mutable_identity() = identity("r1");
  acquire.set_generation(1);
  ASSERT_TRUE(client_->call<wire::AcquireControlResponse>(acquire,
      &wire::RobotAdapterServerService::Stub::AcquireControl).status.ok());
  auto motion = command("r1", "motion", "moving");
  motion.set_controller_grant(install.controller_grant());
  motion.set_control_generation(1);
  ASSERT_EQ(operation::OPERATION_PHASE_SUCCEEDED, client_->execute(motion).response.event().phase());
  world_.fail_native_retire = true;
  wire::RetireControllerRequest retire;
  retire.set_process_owner(kTestProcessOwner);
  retire.set_generation(1);
  retire.set_open_id(install.open_id());
  auto first = client_->call<wire::RetireControllerResponse>(retire,
      &wire::RobotAdapterServerService::Stub::RetireController);
  EXPECT_EQ(grpc::StatusCode::UNKNOWN, first.status.error_code());
  EXPECT_FALSE(first.response.joined());
  EXPECT_TRUE(world_.contains("retire-failed:r1"));
  auto successor = install;
  successor.set_generation(2);
  successor.set_open_id(std::string(16, 'n'));
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION,
      client_->call<wire::InstallControllerResponse>(successor,
          &wire::RobotAdapterServerService::Stub::InstallController).status.error_code());
  world_.fail_native_retire = false;
  const auto joined = client_->call<wire::RetireControllerResponse>(retire,
      &wire::RobotAdapterServerService::Stub::RetireController);
  ASSERT_TRUE(joined.status.ok()) << joined.status.error_message();
  EXPECT_TRUE(joined.response.joined());
  EXPECT_TRUE(world_.contains("retire-control:r1"));
  EXPECT_TRUE(control.waitFinished());
  EXPECT_TRUE(client_->call<wire::InstallControllerResponse>(successor,
      &wire::RobotAdapterServerService::Stub::InstallController).status.ok());
}

// These use the real native gRPC server and its normal four status slots.
// The held fake calls are fault injection, not a hardware safety claim.
TEST_F(ServerController, TwoBlockedMemberRetirementsDoNotStarveOtherSafetyOrShutdown) {
  auto timings = quickTimings();
  timings.member_drain = milliseconds(100);
  timings.status_flush = milliseconds(30);
  start(timings);
  const auto described = client_->describe(10000);
  ASSERT_TRUE(described.status.ok());
  wire::ApplyMembersRequest add;
  for (const auto &id : {"r1", "r2", "r3"})
    *add.add_members() = registration(id, std::string("/fleet/") + id);
  ASSERT_TRUE(client_->apply(add).status.ok());
  wire::InstallControllerRequest install;
  install.set_process_owner(kTestProcessOwner);
  install.set_generation(1);
  install.set_open_id(std::string(16, 'i'));
  install.set_controller_grant(std::string(32, 'g'));
  install.mutable_binding()->set_provider_instance_id(described.response.service_ref().instance_id());
  install.mutable_binding()->set_core_id("core");
  install.mutable_binding()->set_core_boot_id("boot");
  ASSERT_TRUE(client_->call<wire::InstallControllerResponse>(install,
      &wire::RobotAdapterServerService::Stub::InstallController).status.ok());
  wire::SubscribeStatusRequest open;
  open.set_control(true);
  open.mutable_owner()->set_controller_grant(install.controller_grant());
  open.mutable_owner()->set_generation(1);
  open.mutable_owner()->set_open_id(std::string(16, 'a'));
  Client::Watch control(*client_, milliseconds(20000), open);
  ASSERT_TRUE(control.wait([](const wire::SubscribeStatusResponse &frame) { return frame.generation() == 1; }));
  for (const auto &id : {"r1", "r2", "r3"}) {
    wire::AcquireControlRequest acquire;
    acquire.set_controller_grant(install.controller_grant());
    *acquire.mutable_identity() = identity(id);
    acquire.set_generation(1);
    ASSERT_TRUE(client_->call<wire::AcquireControlResponse>(acquire,
        &wire::RobotAdapterServerService::Stub::AcquireControl).status.ok());
  }
  auto third = command("r3", "motion", "third-motion");
  third.set_controller_grant(install.controller_grant());
  third.set_control_generation(1);
  ASSERT_EQ(operation::OPERATION_PHASE_SUCCEEDED, client_->execute(third).response.event().phase());
  auto launchSlow = [&](const std::string &id) {
    auto request = command(id, "slow-motion", id + "-slow");
    request.set_controller_grant(install.controller_grant());
    request.set_control_generation(1);
    return std::async(std::launch::async, [this, request] {
      Client actor(socket());
      return actor.execute(request, milliseconds(15000));
    });
  };
  auto motion1 = launchSlow("r1");
  auto motion2 = launchSlow("r2");
  for (int attempt = 0; attempt < 400 &&
       (!world_.contains("execute:r1:slow-motion") || !world_.contains("execute:r2:slow-motion")); ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  ASSERT_TRUE(world_.contains("execute:r1:slow-motion"));
  ASSERT_TRUE(world_.contains("execute:r2:slow-motion"));
  auto releaseRequest = [&](const std::string &id) {
    wire::ReleaseControlRequest request;
    request.set_controller_grant(install.controller_grant());
    *request.mutable_identity() = identity(id);
    request.set_generation(1);
    return request;
  };
  auto launchRelease = [&](const std::string &id) {
    auto request = releaseRequest(id);
    return std::async(std::launch::async, [this, request] {
      Client owner(socket());
      return owner.call<wire::ReleaseControlResponse>(request,
          &wire::RobotAdapterServerService::Stub::ReleaseControl, milliseconds(12000));
    });
  };
  auto release1 = launchRelease("r1");
  auto release2 = launchRelease("r2");
  for (int attempt = 0; attempt < 400 && server_->SnapshotStatusWork().retirement_waiters != 2; ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  EXPECT_EQ(2u, server_->SnapshotStatusWork().retirement_waiters);
  EXPECT_EQ(std::future_status::timeout, release1.wait_for(milliseconds(150)));
  EXPECT_EQ(std::future_status::timeout, release2.wait_for(milliseconds(150)));
  const auto duplicate = client_->call<wire::ReleaseControlResponse>(releaseRequest("r1"),
      &wire::RobotAdapterServerService::Stub::ReleaseControl, milliseconds(500));
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION, duplicate.status.error_code());
  auto foreign = releaseRequest("r3");
  foreign.set_controller_grant(std::string(32, 'x'));
  EXPECT_EQ(grpc::StatusCode::PERMISSION_DENIED,
            client_->call<wire::ReleaseControlResponse>(foreign,
                &wire::RobotAdapterServerService::Stub::ReleaseControl, milliseconds(500)).status.error_code());
  EXPECT_EQ(0u, server_->SnapshotStatusWork().safety_work);
  const auto independent = client_->call<wire::ReleaseControlResponse>(releaseRequest("r3"),
      &wire::RobotAdapterServerService::Stub::ReleaseControl, milliseconds(2000));
  EXPECT_TRUE(independent.status.ok()) << independent.status.error_message();
  EXPECT_TRUE(independent.response.joined());
  EXPECT_TRUE(world_.contains("retire-control:r3"));
  wire::ApplyMembersRequest fourth;
  *fourth.add_members() = registration("r4", "/fleet/r4");
  const auto added = client_->apply(fourth);
  EXPECT_TRUE(added.status.ok());
  EXPECT_EQ(1, added.response.members_size());
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED,
            client_->execute(command("r4", "instant", "unrelated-live")).response.event().phase());

  // A timed-out native Stop is still physically owned. A Shutdown receipt
  // cannot claim success before that exact Stop and its publication join.
  world_.holdStop("r3");
  auto shutdown = std::async(std::launch::async, [this] {
    Client stopper(socket());
    return stopper.shutdown(milliseconds(10000));
  });
  const bool entered = world_.waitStopEntered("r3", milliseconds(4000));
  EXPECT_TRUE(entered);
  if (entered) {
    EXPECT_EQ(std::future_status::timeout, shutdown.wait_for(milliseconds(250)))
        << "Shutdown answered before the held native Stop joined";
  }
  world_.releaseStop("r3");
  const auto stopped = shutdown.get();
  EXPECT_TRUE(stopped.status.ok()) << stopped.status.error_message();
  EXPECT_TRUE(world_.contains("stop:r3"));
  (void)motion1.get();
  (void)motion2.get();
  (void)release1.get();
  (void)release2.get();
  EXPECT_EQ(kExitOk, join());
}

TEST_F(ServerController, TwoControllerRetirementsStillAllowIndependentSafetyRelease) {
  startReady();
  const auto described = client_->describe();
  wire::ApplyMembersRequest add;
  *add.add_members() = registration("r1", "/fleet/r1");
  *add.add_members() = registration("r2", "/fleet/r2");
  ASSERT_TRUE(client_->apply(add).status.ok());
  wire::InstallControllerRequest install;
  install.set_process_owner(kTestProcessOwner);
  install.set_generation(1);
  install.set_open_id(std::string(16, 'i'));
  install.set_controller_grant(std::string(32, 'g'));
  install.mutable_binding()->set_provider_instance_id(described.response.service_ref().instance_id());
  install.mutable_binding()->set_core_id("core");
  install.mutable_binding()->set_core_boot_id("boot");
  ASSERT_TRUE(client_->call<wire::InstallControllerResponse>(install,
      &wire::RobotAdapterServerService::Stub::InstallController).status.ok());
  wire::SubscribeStatusRequest open;
  open.set_control(true);
  open.mutable_owner()->set_controller_grant(install.controller_grant());
  open.mutable_owner()->set_generation(1);
  open.mutable_owner()->set_open_id(std::string(16, 'a'));
  Client::Watch control(*client_, milliseconds(20000), open);
  ASSERT_TRUE(control.wait([](const wire::SubscribeStatusResponse &frame) { return frame.generation() == 1; }));
  for (const auto &id : {"r1", "r2"}) {
    wire::AcquireControlRequest acquire;
    acquire.set_controller_grant(install.controller_grant());
    *acquire.mutable_identity() = identity(id);
    acquire.set_generation(1);
    ASSERT_TRUE(client_->call<wire::AcquireControlResponse>(acquire,
        &wire::RobotAdapterServerService::Stub::AcquireControl).status.ok());
  }
  auto slow = command("r1", "slow-motion", "slow-motion-work");
  slow.set_controller_grant(install.controller_grant());
  slow.set_control_generation(1);
  auto motion = std::async(std::launch::async, [this, slow] {
    Client actor(socket());
    return actor.execute(slow, milliseconds(15000));
  });
  for (int attempt = 0; attempt < 200 && !world_.contains("execute:r1:slow-motion"); ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  ASSERT_TRUE(world_.contains("execute:r1:slow-motion"));
  auto second = command("r2", "motion", "independent-motion");
  second.set_controller_grant(install.controller_grant());
  second.set_control_generation(1);
  ASSERT_EQ(operation::OPERATION_PHASE_SUCCEEDED, client_->execute(second).response.event().phase());
  wire::RetireControllerRequest retire;
  retire.set_process_owner(kTestProcessOwner);
  retire.set_generation(1);
  retire.set_open_id(install.open_id());
  auto launchRetire = [&] {
    return std::async(std::launch::async, [this, retire] {
      Client owner(socket());
      return owner.call<wire::RetireControllerResponse>(retire,
          &wire::RobotAdapterServerService::Stub::RetireController, milliseconds(12000));
    });
  };
  auto first = launchRetire();
  auto duplicate = launchRetire();
  for (int attempt = 0; attempt < 400 && server_->SnapshotStatusWork().retirement_waiters != 2; ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  EXPECT_EQ(2u, server_->SnapshotStatusWork().retirement_waiters);
  EXPECT_EQ(std::future_status::timeout, first.wait_for(milliseconds(150)));
  EXPECT_EQ(std::future_status::timeout, duplicate.wait_for(milliseconds(150)));
  const auto safety = client_->execute(command("r2", "release", "safe-r2"), milliseconds(2000));
  EXPECT_TRUE(safety.status.ok()) << safety.status.error_message();
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, safety.response.event().phase());
  EXPECT_TRUE(world_.contains("execute:r2:release"));
  wire::ReleaseControlRequest release;
  release.set_controller_grant(std::string(32, 'x'));
  *release.mutable_identity() = identity("r2");
  release.set_generation(1);
  auto denied = client_->call<wire::ReleaseControlResponse>(release,
      &wire::RobotAdapterServerService::Stub::ReleaseControl, milliseconds(2000));
  EXPECT_EQ(grpc::StatusCode::PERMISSION_DENIED, denied.status.error_code());
  release.set_controller_grant(install.controller_grant());
  auto independent = client_->call<wire::ReleaseControlResponse>(release,
      &wire::RobotAdapterServerService::Stub::ReleaseControl, milliseconds(2000));
  EXPECT_TRUE(independent.status.ok()) << independent.status.error_message();
  EXPECT_TRUE(independent.response.joined());
  wire::ApplyMembersRequest another;
  *another.add_members() = registration("r3", "/fleet/r3");
  EXPECT_TRUE(client_->apply(another).status.ok());
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED,
            client_->execute(command("r3", "instant", "unrelated-live")).response.event().phase());
  auto successor = install;
  successor.set_generation(2);
  successor.set_open_id(std::string(16, 'n'));
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION,
      client_->call<wire::InstallControllerResponse>(successor,
          &wire::RobotAdapterServerService::Stub::InstallController).status.error_code());
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.complete_slow_motion = true;
  }
  (void)motion.get();
  const auto joined1 = first.get();
  const auto joined2 = duplicate.get();
  EXPECT_TRUE(joined1.status.ok());
  EXPECT_TRUE(joined1.response.joined());
  EXPECT_TRUE(joined2.status.ok());
  EXPECT_TRUE(joined2.response.joined());
  EXPECT_TRUE(control.waitFinished());
  EXPECT_TRUE(client_->call<wire::InstallControllerResponse>(successor,
      &wire::RobotAdapterServerService::Stub::InstallController).status.ok());
}



TEST_F(ServerController, TwoBlockedNativeRetirementsKeepIndependentSafetyAndShutdown) {
  startReady();
  const auto described = client_->describe();
  wire::ApplyMembersRequest add;
  for (const auto &id : {"r1", "r2", "r3"})
    *add.add_members() = registration(id, std::string("/fleet/") + id);
  ASSERT_TRUE(client_->apply(add).status.ok());
  wire::InstallControllerRequest install;
  install.set_process_owner(kTestProcessOwner);
  install.set_generation(1);
  install.set_open_id(std::string(16, 'i'));
  install.set_controller_grant(std::string(32, 'g'));
  install.mutable_binding()->set_provider_instance_id(described.response.service_ref().instance_id());
  install.mutable_binding()->set_core_id("core");
  install.mutable_binding()->set_core_boot_id("boot");
  ASSERT_TRUE(client_->call<wire::InstallControllerResponse>(install,
      &wire::RobotAdapterServerService::Stub::InstallController).status.ok());
  wire::SubscribeStatusRequest open;
  open.set_control(true);
  open.mutable_owner()->set_controller_grant(install.controller_grant());
  open.mutable_owner()->set_generation(1);
  open.mutable_owner()->set_open_id(std::string(16, 'a'));
  Client::Watch control(*client_, milliseconds(20000), open);
  ASSERT_TRUE(control.wait([](const wire::SubscribeStatusResponse &frame) { return frame.generation() == 1; }));
  for (const auto &id : {"r1", "r2", "r3"}) {
    wire::AcquireControlRequest acquire;
    acquire.set_controller_grant(install.controller_grant());
    *acquire.mutable_identity() = identity(id);
    acquire.set_generation(1);
    ASSERT_TRUE(client_->call<wire::AcquireControlResponse>(acquire,
        &wire::RobotAdapterServerService::Stub::AcquireControl).status.ok());
    auto motion = command(id, "motion", std::string(id) + "-motion");
    motion.set_controller_grant(install.controller_grant());
    motion.set_control_generation(1);
    ASSERT_EQ(operation::OPERATION_PHASE_SUCCEEDED, client_->execute(motion).response.event().phase());
  }
  auto releaseRequest = [&](const std::string &id) {
    wire::ReleaseControlRequest release;
    release.set_controller_grant(install.controller_grant());
    *release.mutable_identity() = identity(id);
    release.set_generation(1);
    return release;
  };
  auto launch = [&](const std::string &id) {
    auto release = releaseRequest(id);
    return std::async(std::launch::async, [this, release] {
      Client owner(socket());
      return owner.call<wire::ReleaseControlResponse>(release,
          &wire::RobotAdapterServerService::Stub::ReleaseControl, milliseconds(12000));
    });
  };
  world_.holdRetire("r1");
  world_.holdRetire("r2");
  auto first = launch("r1");
  auto second = launch("r2");
  EXPECT_TRUE(world_.waitRetireEntered("r1", milliseconds(4000)));
  EXPECT_TRUE(world_.waitRetireEntered("r2", milliseconds(4000)));
  EXPECT_EQ(2u, server_->SnapshotStatusWork().retirement_waiters);
  EXPECT_EQ(std::future_status::timeout, first.wait_for(milliseconds(100)));
  EXPECT_EQ(std::future_status::timeout, second.wait_for(milliseconds(100)));
  auto independent = client_->call<wire::ReleaseControlResponse>(releaseRequest("r3"),
      &wire::RobotAdapterServerService::Stub::ReleaseControl, milliseconds(2000));
  EXPECT_TRUE(independent.status.ok()) << independent.status.error_message();
  EXPECT_TRUE(independent.response.joined());
  EXPECT_TRUE(world_.contains("retire-control:r3"));
  wire::ApplyMembersRequest fourth;
  *fourth.add_members() = registration("r4", "/fleet/r4");
  EXPECT_TRUE(client_->apply(fourth).status.ok());
  const auto unrelated = client_->execute(command("r4", "instant", "native-retire-unrelated"), milliseconds(2000));
  EXPECT_TRUE(unrelated.status.ok());
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, unrelated.response.event().phase());
  auto shutdown = std::async(std::launch::async, [this] {
    Client stopper(socket());
    return stopper.shutdown(milliseconds(10000));
  });
  bool draining = false;
  for (int attempt = 0; attempt < 200 && !draining; ++attempt) {
    const auto health = client_->health();
    draining = health.status.ok() && !health.response.serving();
    if (!draining) std::this_thread::sleep_for(milliseconds(5));
  }
  EXPECT_TRUE(draining);
  EXPECT_EQ(std::future_status::timeout, shutdown.wait_for(milliseconds(200)));
  EXPECT_FALSE(world_.contains("retire-control:r1"));
  EXPECT_FALSE(world_.contains("retire-control:r2"));
  world_.releaseRetire("r1");
  world_.releaseRetire("r2");
  EXPECT_TRUE(first.get().response.joined());
  EXPECT_TRUE(second.get().response.joined());
  const auto stopped = shutdown.get();
  EXPECT_TRUE(stopped.status.ok()) << stopped.status.error_message();
  EXPECT_TRUE(world_.contains("stop:r1"));
  EXPECT_TRUE(world_.contains("stop:r2"));
  EXPECT_EQ(kExitOk, join());
}

TEST_F(ServerController, NativeControllerJoinDoesNotBlockAnotherMemberSafety) {
  startReady();
  const auto described = client_->describe();
  wire::ApplyMembersRequest add;
  for (const auto &id : {"r1", "r2"})
    *add.add_members() = registration(id, std::string("/fleet/") + id);
  ASSERT_TRUE(client_->apply(add).status.ok());
  wire::InstallControllerRequest install;
  install.set_process_owner(kTestProcessOwner);
  install.set_generation(1);
  install.set_open_id(std::string(16, 'i'));
  install.set_controller_grant(std::string(32, 'g'));
  install.mutable_binding()->set_provider_instance_id(described.response.service_ref().instance_id());
  install.mutable_binding()->set_core_id("core");
  install.mutable_binding()->set_core_boot_id("boot");
  ASSERT_TRUE(client_->call<wire::InstallControllerResponse>(install,
      &wire::RobotAdapterServerService::Stub::InstallController).status.ok());
  wire::SubscribeStatusRequest open;
  open.set_control(true);
  open.mutable_owner()->set_controller_grant(install.controller_grant());
  open.mutable_owner()->set_generation(1);
  open.mutable_owner()->set_open_id(std::string(16, 'a'));
  Client::Watch control(*client_, milliseconds(20000), open);
  ASSERT_TRUE(control.wait([](const wire::SubscribeStatusResponse &frame) { return frame.generation() == 1; }));
  for (const auto &id : {"r1", "r2"}) {
    wire::AcquireControlRequest acquire;
    acquire.set_controller_grant(install.controller_grant());
    *acquire.mutable_identity() = identity(id);
    acquire.set_generation(1);
    ASSERT_TRUE(client_->call<wire::AcquireControlResponse>(acquire,
        &wire::RobotAdapterServerService::Stub::AcquireControl).status.ok());
    auto motion = command(id, "motion", std::string(id) + "-motion");
    motion.set_controller_grant(install.controller_grant());
    motion.set_control_generation(1);
    ASSERT_EQ(operation::OPERATION_PHASE_SUCCEEDED, client_->execute(motion).response.event().phase());
  }
  world_.holdRetire("r1");
  wire::RetireControllerRequest retire;
  retire.set_process_owner(kTestProcessOwner);
  retire.set_generation(1);
  retire.set_open_id(install.open_id());
  auto launch = [&] {
    return std::async(std::launch::async, [this, retire] {
      Client owner(socket());
      return owner.call<wire::RetireControllerResponse>(retire,
          &wire::RobotAdapterServerService::Stub::RetireController, milliseconds(12000));
    });
  };
  auto first = launch();
  const bool entered = world_.waitRetireEntered("r1", milliseconds(4000));
  EXPECT_TRUE(entered);
  auto second = launch();
  for (int attempt = 0; attempt < 400 && server_->SnapshotStatusWork().retirement_waiters != 2; ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  EXPECT_EQ(2u, server_->SnapshotStatusWork().retirement_waiters);
  EXPECT_EQ(std::future_status::timeout, first.wait_for(milliseconds(100)));
  EXPECT_EQ(std::future_status::timeout, second.wait_for(milliseconds(100)));
  auto same_member = std::async(std::launch::async, [this] {
    Client actor(socket());
    return actor.execute(command("r1", "instant", "queued-until-native-join"), milliseconds(10000));
  });
  EXPECT_EQ(std::future_status::timeout, same_member.wait_for(milliseconds(100)));
  const auto safety = client_->execute(command("r2", "release", "other-member-safety"), milliseconds(2000));
  EXPECT_TRUE(safety.status.ok());
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, safety.response.event().phase());
  wire::ReleaseControlRequest release;
  release.set_controller_grant(install.controller_grant());
  *release.mutable_identity() = identity("r2");
  release.set_generation(1);
  const auto independent = client_->call<wire::ReleaseControlResponse>(release,
      &wire::RobotAdapterServerService::Stub::ReleaseControl, milliseconds(2000));
  EXPECT_TRUE(independent.status.ok()) << independent.status.error_message();
  EXPECT_TRUE(independent.response.joined());
  wire::ApplyMembersRequest another;
  *another.add_members() = registration("r3", "/fleet/r3");
  EXPECT_TRUE(client_->apply(another).status.ok());
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED,
            client_->execute(command("r3", "instant", "other-live-member"), milliseconds(2000)).response.event().phase());
  auto successor = install;
  successor.set_generation(2);
  successor.set_open_id(std::string(16, 'n'));
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION,
      client_->call<wire::InstallControllerResponse>(successor,
          &wire::RobotAdapterServerService::Stub::InstallController).status.error_code());
  world_.releaseRetire("r1");
  EXPECT_TRUE(first.get().response.joined());
  EXPECT_TRUE(second.get().response.joined());
  const auto resumed = same_member.get();
  EXPECT_TRUE(resumed.status.ok());
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, resumed.response.event().phase());
  EXPECT_TRUE(world_.contains("retire-control:r1"));
  EXPECT_TRUE(client_->call<wire::InstallControllerResponse>(successor,
      &wire::RobotAdapterServerService::Stub::InstallController).status.ok());
}

TEST_F(ServerController, FullOrdinaryWorkAndTwoRetirementsKeepSafetyMembershipAndShutdown) {
  start(ServerTimings{});
  const auto described = client_->describe(10000);
  ASSERT_TRUE(described.status.ok());
  ASSERT_TRUE(described.response.ready());
  wire::ApplyMembersRequest add;
  for (int index = 1; index <= 19; ++index) {
    const auto id = "r" + std::to_string(index);
    *add.add_members() = registration(id, "/fleet/" + id);
  }
  ASSERT_TRUE(client_->apply(add).status.ok());
  wire::InstallControllerRequest install;
  install.set_process_owner(kTestProcessOwner);
  install.set_generation(1);
  install.set_open_id(std::string(16, 'i'));
  install.set_controller_grant(std::string(32, 'g'));
  install.mutable_binding()->set_provider_instance_id(described.response.service_ref().instance_id());
  install.mutable_binding()->set_core_id("core");
  install.mutable_binding()->set_core_boot_id("boot");
  ASSERT_TRUE(client_->call<wire::InstallControllerResponse>(install,
      &wire::RobotAdapterServerService::Stub::InstallController).status.ok());
  wire::SubscribeStatusRequest open;
  open.set_control(true);
  open.mutable_owner()->set_controller_grant(install.controller_grant());
  open.mutable_owner()->set_generation(1);
  open.mutable_owner()->set_open_id(std::string(16, 'a'));
  Client::Watch control(*client_, milliseconds(20000), open);
  ASSERT_TRUE(control.wait([](const wire::SubscribeStatusResponse &frame) { return frame.generation() == 1; }));
  Client::Watch view1(*client_, milliseconds(20000), Client::Watch::broadSelection());
  Client::Watch view2(*client_, milliseconds(20000), Client::Watch::broadSelection());
  Client::Watch view3(*client_, milliseconds(20000), Client::Watch::broadSelection());
  for (const auto &id : {"r1", "r2", "r19"}) {
    wire::AcquireControlRequest acquire;
    acquire.set_controller_grant(install.controller_grant());
    *acquire.mutable_identity() = identity(id);
    acquire.set_generation(1);
    ASSERT_TRUE(client_->call<wire::AcquireControlResponse>(acquire,
        &wire::RobotAdapterServerService::Stub::AcquireControl).status.ok());
  }
  auto ownMotion = command("r19", "motion", "r19-motion");
  ownMotion.set_controller_grant(install.controller_grant());
  ownMotion.set_control_generation(1);
  ASSERT_EQ(operation::OPERATION_PHASE_SUCCEEDED, client_->execute(ownMotion).response.event().phase());
  std::vector<std::future<Reply<wire::ExecuteResponse>>> commands;
  for (int index = 1; index <= 18; ++index) {
    const auto id = "r" + std::to_string(index);
    auto request = command(id, index <= 2 ? "slow-motion" : "slow", id + "-work");
    if (index <= 2) {
      request.set_controller_grant(install.controller_grant());
      request.set_control_generation(1);
    }
    commands.push_back(std::async(std::launch::async, [this, request] {
      Client actor(socket());
      return actor.execute(request, milliseconds(15000));
    }));
  }
  for (int attempt = 0; attempt < 800 &&
       (server_->SnapshotStatusWork().ordinary_work != 18 ||
        !world_.contains("execute:r1:slow-motion") || !world_.contains("execute:r2:slow-motion")); ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  EXPECT_EQ(18u, server_->SnapshotStatusWork().ordinary_work);
  EXPECT_TRUE(world_.contains("execute:r1:slow-motion"));
  EXPECT_TRUE(world_.contains("execute:r2:slow-motion"));
  auto launchRelease = [&](const std::string &id) {
    wire::ReleaseControlRequest request;
    request.set_controller_grant(install.controller_grant());
    *request.mutable_identity() = identity(id);
    request.set_generation(1);
    return std::async(std::launch::async, [this, request] {
      Client owner(socket());
      return owner.call<wire::ReleaseControlResponse>(request,
          &wire::RobotAdapterServerService::Stub::ReleaseControl, milliseconds(12000));
    });
  };
  auto release1 = launchRelease("r1");
  auto release2 = launchRelease("r2");
  for (int attempt = 0; attempt < 400 && server_->SnapshotStatusWork().retirement_waiters != 2; ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  EXPECT_EQ(2u, server_->SnapshotStatusWork().retirement_waiters);
  EXPECT_EQ(4u, server_->SnapshotStatusWork().subscribers);
  const auto safety = client_->execute(command("r19", "release", "safety-r19"), milliseconds(2000));
  EXPECT_TRUE(safety.status.ok()) << safety.status.error_message();
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, safety.response.event().phase());
  EXPECT_TRUE(world_.contains("execute:r19:release"));
  wire::ApplyMembersRequest additional;
  *additional.add_members() = registration("r20", "/fleet/r20");
  const auto added = client_->apply(additional);
  EXPECT_TRUE(added.status.ok()) << added.status.error_message();
  EXPECT_EQ(1, added.response.members_size());
  const auto shutdown = client_->shutdown(milliseconds(10000));
  EXPECT_TRUE(shutdown.status.ok()) << shutdown.status.error_message();
  if (!shutdown.status.ok()) server_->RequestStop();
  for (auto &pending : commands) (void)pending.get();
  (void)release1.get();
  (void)release2.get();
  EXPECT_EQ(kExitOk, join());
  EXPECT_TRUE(world_.contains("stop:r19"));
  EXPECT_TRUE(world_.contains("stop:r20"));
}

TEST_F(ServerController, CancelledCallerRetainsActualNativeWorkCharge) {
  startReady();
  wire::ApplyMembersRequest add;
  *add.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(add).status.ok());
  world_.hold_cancelled_native = true;
  auto executing = std::async(std::launch::async, [this] {
    Client actor(socket());
    return actor.execute(command("r1", "slow", "held-cancel"), milliseconds(500));
  });
  struct Restore {
    World &world;
    ~Restore() { world.hold_cancelled_native = false; }
  } restore{world_};
  for (int attempt = 0; attempt < 200 && !world_.contains("execute:r1:slow"); ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  ASSERT_TRUE(world_.contains("execute:r1:slow"));
  const auto answer = executing.get();
  EXPECT_FALSE(answer.status.ok());
  EXPECT_FALSE(world_.contains("cancelled:held-cancel"));
  EXPECT_EQ(1u, server_->SnapshotStatusWork().ordinary_work)
      << "caller cancellation refunded a live native work permit";
  EXPECT_TRUE(client_->health().status.ok());
  world_.hold_cancelled_native = false;
  for (int attempt = 0; attempt < 400 && server_->SnapshotStatusWork().ordinary_work != 0; ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  EXPECT_TRUE(world_.contains("cancelled:held-cancel"));
  EXPECT_EQ(0u, server_->SnapshotStatusWork().ordinary_work);
}

TEST_F(ServerNetwork, TlsAndUnixShareIdentityTelemetryOperationsAndShutdown) {
  TlsFixture network;
  start(quickTimings(), network.bootstrap());
  const auto local = client_->describe(10000);
  ASSERT_TRUE(local.status.ok()) << local.status.error_message();
  ASSERT_TRUE(local.response.ready());
  const auto local_facts = facts(local.response);
  ASSERT_TRUE(local_facts.contains("direct_service_ref"));
  const auto &direct = local_facts.at("direct_service_ref");
  EXPECT_EQ("local", direct.at("target_id"));
  EXPECT_EQ(local.response.service_ref().service(), direct.at("service"));
  EXPECT_EQ("v2", direct.at("api_version"));
  EXPECT_EQ("grpc.v1", direct.at("profile"));
  EXPECT_EQ("tls", direct.at("endpoint").at("kind"));
  EXPECT_EQ("127.0.0.1:" + std::to_string(network.port), direct.at("endpoint").at("address"));
  const auto instance = direct.at("instance_id").get<std::string>();
  EXPECT_EQ(local.response.service_ref().instance_id(), instance);
  EXPECT_EQ("unix", local.response.service_ref().endpoint().kind());
  EXPECT_EQ(socket(), local.response.service_ref().endpoint().address());

  const auto channel = network.channel(direct.at("endpoint").at("address").get<std::string>());
  Client remote(channel, "Bearer test-gateway-token", instance);
  const auto described = remote.describe(10000);
  ASSERT_TRUE(described.status.ok()) << described.status.error_message();
  ASSERT_TRUE(described.response.ready());
  EXPECT_EQ(local.response.service_ref().SerializeAsString(), described.response.service_ref().SerializeAsString());
  EXPECT_EQ(direct, facts(described.response).at("direct_service_ref"));

  wire::ApplyMembersRequest add;
  *add.add_members() = registration("robot-1", "/robot_1");
  const auto applied = client_->apply(add);
  ASSERT_TRUE(applied.status.ok());
  ASSERT_EQ(1, applied.response.members_size());
  ASSERT_FALSE(applied.response.members(0).has_error());
  Client::Watch watch(remote, milliseconds(10000), Client::Watch::broadSelection());
  ASSERT_TRUE(watch.wait([](const wire::SubscribeStatusResponse &frame) {
    for (const auto &member : frame.members())
      if (member.identity().robot_id() == "robot-1" &&
          member.identity().target_id() == "local" &&
          member.identity().run_id() == "run-test" &&
          member.identity().connection_epoch() == 1)
        for (const auto &message : member.messages())
          if (message.channel_id() == "state.test" && message.message().sequence() > 0)
            return true;
    return false;
  }));
  const auto executed = remote.execute(command("robot-1", "instant", "remote-work"));
  ASSERT_TRUE(executed.status.ok()) << executed.status.error_message();
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, executed.response.event().phase());
  auto calls = world_.snapshot();
  EXPECT_EQ(1, std::count(calls.begin(), calls.end(), "execute:robot-1:instant"));

  Client unauthorized(channel, "Bearer wrong-token", instance);
  const auto refused = unauthorized.execute(command("robot-1", "instant", "forbidden-work"));
  EXPECT_EQ(grpc::StatusCode::UNAUTHENTICATED, refused.status.error_code());
  calls = world_.snapshot();
  EXPECT_EQ(1, std::count(calls.begin(), calls.end(), "execute:robot-1:instant"));
  const auto stopped = client_->shutdown();
  ASSERT_TRUE(stopped.status.ok()) << stopped.status.error_message();
  EXPECT_TRUE(watch.waitFinished());
  EXPECT_EQ(kExitOk, join());
  EXPECT_TRUE(world_.contains("stop:robot-1"));
  const auto after = remote.call<wire::HealthResponse>(wire::HealthRequest(),
      &wire::RobotAdapterServerService::Stub::Health, milliseconds(250));
  EXPECT_FALSE(after.status.ok());
}

TEST_F(ServerNetwork, PlainUnixDoesNotAdvertiseADirectService) {
  startReady();
  const auto described = client_->describe();
  ASSERT_TRUE(described.status.ok());
  EXPECT_EQ("unix", described.response.service_ref().endpoint().kind());
  EXPECT_FALSE(facts(described.response).contains("direct_service_ref"));
  ASSERT_TRUE(client_->shutdown().status.ok());
  EXPECT_EQ(kExitOk, join());
}

TEST_F(ServerNetwork, RejectsNetworkBootstrapForAnotherTargetOrService) {
  TlsFixture network;
  for (const auto &path : {network.bootstrap("another-target"),
                           network.bootstrap("local", "another.Service"),
                           network.bootstrap("local", "xgc.robot.v1.RobotAdapterServerService", "v1")}) {
    auto configured = options();
    configured.network_bootstrap = path;
    EXPECT_THROW({ RobotServer server(std::move(configured), *binding_, factoryFor(world_)); },
                 std::invalid_argument);
  }
  EXPECT_TRUE(world_.snapshot().empty());
}

TEST(MasterRetry, UsesFullJitterExponentialCeilingsAndResetsAfterSuccess) {
  MasterRetry retry;
  auto ceiling = milliseconds(250);
  std::set<long long> capped_delays;
  for (int attempt = 0; attempt < 40; ++attempt) {
    const auto delay = retry.failed();
    EXPECT_GE(delay, milliseconds(1));
    EXPECT_LE(delay, ceiling);
    if (ceiling == milliseconds(10000))
      capped_delays.insert(delay.count());
    ceiling = std::min(ceiling * 2, milliseconds(10000));
  }
  EXPECT_GT(capped_delays.size(), 1u);
  retry.reset();
  EXPECT_LE(retry.failed(), milliseconds(250));
}

TEST_F(ServerLifecycle, DescribeHoldsUntilTheRosNodeIsRegisteredThenReportsTheFacts) {
  binding_->hold_connect = true;
  start();
  const auto early = client_->describe(0);
  ASSERT_TRUE(early.status.ok());
  EXPECT_FALSE(early.response.ready());
  EXPECT_EQ("starting", facts(early.response)["state"]);
  EXPECT_EQ("xgc.robot.v1.RobotAdapterServerService", early.response.service_ref().service());
  EXPECT_EQ("v2", early.response.service_ref().api_version());
  EXPECT_EQ("fake-provider", early.response.provider_definition_id());

  std::atomic<bool> answered{false};
  Reply<wire::DescribeResponse> held;
  const auto started = Clock::now();
  std::thread waiter([&] {
    Client second(socket());
    held = second.describe(8000);
    answered = true;
  });
  std::this_thread::sleep_for(milliseconds(250));
  EXPECT_FALSE(answered) << "a held Describe must not answer before readiness";
  binding_->releaseConnect();
  waiter.join();
  EXPECT_LT(Clock::now() - started, milliseconds(6000));
  ASSERT_TRUE(held.status.ok()) << held.status.error_message();
  EXPECT_TRUE(held.response.ready());
  const auto facts_json = facts(held.response);
  EXPECT_EQ("http://127.0.0.1:11311", facts_json["ros_master_uri"]);
  EXPECT_EQ("master-run-1", facts_json["ros_master_run_id"]);
  EXPECT_EQ(4242, facts_json["ros_master_pid"]);
  EXPECT_EQ("/xgc_fake_run_test", facts_json["ros_node"]);
  EXPECT_EQ("run-test", facts_json["run_id"]);
  EXPECT_EQ("ready", facts_json["state"]);
  EXPECT_EQ("xgc_fake_run_test", binding_->initialized_name);
  EXPECT_EQ("http://127.0.0.1:11311", binding_->initialized.ros_master_uri);
}

TEST_F(ServerLifecycle, ADescribeBudgetThatEndsBeforeReadinessAnswersNotReady) {
  binding_->hold_connect = true;
  start();
  const auto started = Clock::now();
  const auto answer = client_->describe(300);
  ASSERT_TRUE(answer.status.ok()) << answer.status.error_message();
  EXPECT_FALSE(answer.response.ready());
  EXPECT_GE(Clock::now() - started, milliseconds(250));
  EXPECT_LT(Clock::now() - started, milliseconds(3000));
  binding_->releaseConnect();
}

TEST_F(ServerLifecycle, DescribeRejectsAWaitBeyondThirtySeconds) {
  startReady();
  const auto answer = client_->describe(30001);
  EXPECT_EQ(grpc::StatusCode::INVALID_ARGUMENT, answer.status.error_code());
  EXPECT_EQ(xgc2::xrpc::Delivery::ResponseReceived, answer.delivery);
}

TEST_F(ServerLifecycle, WorkIsRefusedUntilTheServerIsReady) {
  binding_->hold_connect = true;
  start();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/r1");
  const auto refused = client_->apply(request);
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION, refused.status.error_code());
  EXPECT_EQ(xgc2::xrpc::Delivery::ResponseReceived, refused.delivery);
  const auto health = client_->health();
  ASSERT_TRUE(health.status.ok());
  EXPECT_FALSE(health.response.serving());
  binding_->releaseConnect();
}

using ServerMembers = Fixture;

TEST_F(ServerMembers, ApplyRegistersEveryRobotOfTheRunInOneCallAndReportsFailuresPerMember) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  *request.add_members() = registration("r2", "/fleet/r2");
  *request.add_members() = registration("r3", "/fleet/r3", "rejected.profile");
  *request.add_members() = registration("r4", "/fleet/r1");        // namespace in use
  *request.add_members() = registration("r5", "/fleet/r5", "fake.profile", 1,
                                        "another-run");           // foreign run
  *request.add_members() = registration("r6", "/fleet/r6", "fake.profile", 0);  // no epoch
  auto invalid = registration("r7", "/fleet/r7");
  invalid.mutable_configuration()->clear_parameters();              // no namespace
  *request.add_members() = invalid;
  auto foreign_environment = registration("r8", "/fleet/r8");
  (*foreign_environment.mutable_configuration()->mutable_parameters())
      ["ros_master_uri"] = "http://elsewhere:11311";
  *request.add_members() = foreign_environment;

  const auto applied = client_->apply(request);
  ASSERT_TRUE(applied.status.ok()) << applied.status.error_message();
  ASSERT_EQ(8, applied.response.members_size());
  std::map<std::string, operation::AdapterError> errors;
  for (const auto &member : applied.response.members())
    errors[member.identity().robot_id()] = member.error();
  EXPECT_TRUE(errors["r1"].code().empty());
  EXPECT_TRUE(errors["r2"].code().empty());
  EXPECT_EQ("profile does not match installed native mapping", errors["r3"].message());
  EXPECT_EQ("namespace-conflict", errors["r4"].code());
  EXPECT_NE(std::string::npos, errors["r5"].message().find("another Run"));
  EXPECT_FALSE(errors["r6"].code().empty());
  EXPECT_NE(std::string::npos, errors["r7"].message().find("namespace"));
  EXPECT_NE(std::string::npos, errors["r8"].message().find("ROS environment"));

  const auto health = client_->health();
  ASSERT_TRUE(health.status.ok());
  EXPECT_EQ(2u, health.response.members());
  EXPECT_EQ(2u, health.response.robots());
  EXPECT_TRUE(health.response.serving());
  EXPECT_FALSE(world_.contains("create:r3"));
  const auto events = world_.snapshot();
  EXPECT_EQ(3, std::count(events.begin(), events.end(), "master-tracking"));
}

TEST_F(ServerMembers, ApplyingTheSameMemberAgainChangesNothingAndAConflictingOneIsRefused) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  const auto again = client_->apply(request);
  ASSERT_TRUE(again.status.ok());
  EXPECT_TRUE(again.response.members(0).error().code().empty());
  const auto lines = world_.snapshot();
  EXPECT_EQ(1, std::count(lines.begin(), lines.end(), "create:r1"));

  wire::ApplyMembersRequest changed;
  *changed.add_members() = registration("r1", "/fleet/r1", "other.profile");
  EXPECT_EQ("member-configuration-conflict",
            client_->apply(changed).response.members(0).error().code());
  wire::ApplyMembersRequest other_epoch;
  *other_epoch.add_members() = registration("r1", "/fleet/r1", "fake.profile", 2);
  EXPECT_EQ("member-epoch-conflict",
            client_->apply(other_epoch).response.members(0).error().code());
}

TEST_F(ServerMembers, CancelledNativeWorkJoinsBeforeMemberRemoval) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  *request.add_members() = registration("r2", "/fleet/r2");
  ASSERT_TRUE(client_->apply(request).status.ok());
  world_.hold_cancelled_native = true;
  auto executing = std::async(std::launch::async, [this] {
    Client actor(socket());
    return actor.execute(command("r1", "slow", "removal-held-work"), milliseconds(10000));
  });
  for (int attempt = 0; attempt < 400 && !world_.contains("execute:r1:slow"); ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  EXPECT_TRUE(world_.contains("execute:r1:slow"));
  wire::RemoveMembersRequest removal;
  *removal.add_members() = identity("r1");
  auto removing = std::async(std::launch::async, [this, removal] {
    Client owner(socket());
    return owner.remove(removal);
  });
  EXPECT_EQ(std::future_status::timeout, removing.wait_for(milliseconds(400)));
  EXPECT_FALSE(world_.contains("cancelled:removal-held-work"));
  EXPECT_FALSE(world_.contains("stop:r1"));
  EXPECT_EQ(1u, world_.native_cancel_attempts.load());
  const auto unrelated = client_->execute(command("r2", "instant", "unrelated-during-removal"));
  EXPECT_TRUE(unrelated.status.ok());
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, unrelated.response.event().phase());
  world_.hold_cancelled_native = false;
  const auto removed = removing.get();
  EXPECT_TRUE(removed.status.ok()) << removed.status.error_message();
  ASSERT_EQ(1, removed.response.members_size());
  EXPECT_TRUE(removed.response.members(0).error().code().empty());
  (void)executing.get();
  EXPECT_TRUE(world_.contains("cancelled:removal-held-work"));
  EXPECT_LT(world_.position("cancelled:removal-held-work"), world_.position("stop:r1"));
}

TEST_F(ServerMembers, TimedOutPublicationFlushCannotAcknowledgeRemoval) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  world_.fail_publication_flush = true;
  wire::RemoveMembersRequest removal;
  *removal.add_members() = identity("r1");
  const auto answer = client_->remove(removal);
  EXPECT_EQ(grpc::StatusCode::UNKNOWN, answer.status.error_code());
  EXPECT_TRUE(world_.contains("stop:r1"));
  EXPECT_TRUE(world_.contains("flush-timeout"));
  EXPECT_EQ(kExitFailure, join());
}

TEST_F(ServerMembers, TheStatusStreamShowsMembersAndTheirRemoval) {
  startReady();
  Client::Watch watch(*client_, milliseconds(20000), Client::Watch::broadSelection());
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  *request.add_members() = registration("r2", "/fleet/r2");
  ASSERT_TRUE(client_->apply(request).status.ok());
  for (const char *robot : {"r1", "r2"})
    EXPECT_TRUE(watch.wait([robot](const wire::SubscribeStatusResponse &frame) {
      for (const auto &member : frame.members())
        if (member.identity().robot_id() == robot && member.messages_size() &&
            member.messages(0).channel_id() == "state.test")
          return true;
      return false;
    })) << robot;

  wire::RemoveMembersRequest removal;
  *removal.add_members() = identity("r1");
  const auto removed = client_->remove(removal);
  ASSERT_TRUE(removed.status.ok());
  EXPECT_TRUE(removed.response.members(0).error().code().empty());
  EXPECT_TRUE(watch.wait([](const wire::SubscribeStatusResponse &frame) {
    for (const auto &member : frame.members())
      if (member.identity().robot_id() == "r1" && member.removed())
        return true;
    return false;
  }));
  EXPECT_TRUE(world_.contains("stop:r1"));
  EXPECT_FALSE(world_.contains("stop:r2"));
  // The final command of a stopped robot leaves the process before the robot
  // (and its publishers) is destroyed.
  EXPECT_LT(world_.position("stop:r1"), world_.position("flush"));
  EXPECT_LT(world_.position("flush"), world_.position("destroy:r1"));

  // Removing again, or removing what was never there, is not an error.
  EXPECT_TRUE(client_->remove(removal).response.members(0).error().code().empty());
  wire::RemoveMembersRequest unknown;
  *unknown.add_members() = identity("nobody");
  EXPECT_TRUE(client_->remove(unknown).response.members(0).error().code().empty());
  wire::RemoveMembersRequest wrong_epoch;
  *wrong_epoch.add_members() = identity("r2", 9);
  EXPECT_EQ("member-epoch-conflict",
            client_->remove(wrong_epoch).response.members(0).error().code());
}

TEST_F(ServerMembers, NoStatusDemandDoesNoStatusFrameWork) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  const auto before = world_.telemetryCount("r1", "state.test");
  std::this_thread::sleep_for(milliseconds(600));
  EXPECT_EQ(world_.telemetryCount("r1", "state.test"), before);
  const auto work = server_->SnapshotStatusWork();
  EXPECT_EQ(0u, work.subscribers);
  EXPECT_EQ(0u, work.snapshots);
  EXPECT_EQ(0u, work.writes_started);
  EXPECT_EQ(0u, work.protobuf_bytes);
}

TEST_F(ServerMembers, DescribeReadsCurrentHealthOnlyWhenRequested) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  std::this_thread::sleep_for(milliseconds(300));
  EXPECT_EQ(0, world_.healthReads("r1"));
  EXPECT_EQ(0, world_.telemetryCount("r1", "state.test"));
  const auto described = client_->describe();
  ASSERT_TRUE(described.status.ok());
  EXPECT_EQ("online", facts(described.response)["robots"]["r1"]["state"]);
  EXPECT_EQ(1, world_.healthReads("r1"));
  EXPECT_EQ(0u, server_->SnapshotStatusWork().protobuf_bytes);
}

TEST_F(ServerMembers, IdleNativeCommandIoWaitsForWorkInsteadOfPolling) {
  binding_->real_command_wait = true;
  startReady();
  // With no members the scheduler waits without a periodic deadline. This
  // command must wake it and receive the ordinary unavailable response.
  const auto missing = client_->execute(command("missing", "instant", "missing"), milliseconds(1000));
  ASSERT_TRUE(missing.status.ok()) << missing.status.error_message();
  EXPECT_EQ("member-unavailable", missing.response.event().error().code());

  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  world_.holdTelemetry("r1", "state.test");
  std::this_thread::sleep_for(milliseconds(150));
  const auto before = world_.command_pumps.load();
  std::this_thread::sleep_for(milliseconds(500));
  const auto after = world_.command_pumps.load();
  std::cout << "headless idle real-I/O pump calls in 500ms=" << (after - before)
            << " status subscribers=" << server_->SnapshotStatusWork().subscribers
            << std::endl;
  EXPECT_EQ(before, after) << "an idle native command I/O owner was polled";
  EXPECT_EQ(0u, server_->SnapshotStatusWork().subscribers);

  const auto result = client_->execute(command("r1", "instant", "wake"), milliseconds(1000));
  ASSERT_TRUE(result.status.ok()) << result.status.error_message();
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, result.response.event().phase());
}

TEST_F(ServerMembers, SelectedStatusFiltersMembersChannelsAndJoinsBeforeReplacement) {
  startReady();
  wire::ApplyMembersRequest request;
  auto selected = registration("r1", "/fleet/r1");
  auto *unrelated_channel = selected.mutable_configuration()->add_channels();
  unrelated_channel->set_channel_id("state.zzother");
  unrelated_channel->set_enabled(true);
  *request.add_members() = selected;
  for (int i = 0; i != 20; ++i) {
    const auto name = "other-" + std::to_string(i);
    *request.add_members() = registration(name, "/fleet/" + name);
  }
  const auto applied = client_->apply(request);
  ASSERT_TRUE(applied.status.ok());
  for (const auto &member : applied.response.members())
    ASSERT_FALSE(member.has_error()) << member.error().message();

  wire::SubscribeStatusRequest selection;
  auto *item = selection.mutable_selected()->add_members();
  *item->mutable_identity() = identity("r1");
  item->add_channel_ids("state.test");
  Client::Watch watch(*client_, milliseconds(20000), selection);
  ASSERT_TRUE(watch.wait([](const wire::SubscribeStatusResponse &frame) {
    return frame.members_size() && frame.members(0).messages_size();
  }));
  world_.holdTelemetry("r1", "state.test");
  // Let the one previously admitted selected callback settle. The other
  // channel and the twenty other members keep producing throughout.
  std::this_thread::sleep_for(milliseconds(300));
  const auto before = watch.frames();
  const auto source_before = server_->SnapshotStatusWork();
  const auto other_member_before = world_.telemetryCountForPrefix("other-");
  const auto other_channel_before = world_.telemetryCount("r1", "state.zzother");
  const auto other_ticks_before = world_.periodicCountForPrefix("other-");
  std::this_thread::sleep_for(milliseconds(650));
  const auto after = watch.frames();
  const auto source_after = server_->SnapshotStatusWork();
  EXPECT_GE(world_.periodicCountForPrefix("other-") - other_ticks_before, 20);
  EXPECT_EQ(world_.telemetryCountForPrefix("other-"), other_member_before);
  EXPECT_EQ(world_.telemetryCount("r1", "state.zzother"), other_channel_before);
  EXPECT_EQ(before.size(), after.size());
  EXPECT_EQ(source_before.snapshots, source_after.snapshots);
  EXPECT_EQ(source_before.writes_started, source_after.writes_started);
  EXPECT_EQ(source_before.protobuf_bytes, source_after.protobuf_bytes);
  std::size_t encoded_bytes = 0;
  for (const auto &frame : after) {
    encoded_bytes += frame.ByteSizeLong();
    for (const auto &member : frame.members()) {
      EXPECT_EQ("r1", member.identity().robot_id());
      EXPECT_EQ(1u, member.identity().connection_epoch());
      for (const auto &message : member.messages())
        EXPECT_EQ("state.test", message.channel_id());
    }
  }
  std::cout << "selected protobuf bytes=" << encoded_bytes
            << " frames=" << after.size()
            << " unrelated member periodic calls="
            << world_.periodicCountForPrefix("other-") - other_ticks_before
            << " unrelated member encodes="
            << world_.telemetryCountForPrefix("other-") - other_member_before
            << " unrelated channel encodes="
            << world_.telemetryCount("r1", "state.zzother") - other_channel_before
            << " unrelated native snapshot work="
            << source_after.snapshots - source_before.snapshots
            << std::endl;
  wire::RemoveMembersRequest removal;
  *removal.add_members() = identity("r1");
  ASSERT_TRUE(client_->remove(removal).status.ok());
  ASSERT_TRUE(watch.wait([](const wire::SubscribeStatusResponse &frame) {
    for (const auto &member : frame.members())
      if (member.identity().robot_id() == "r1" && member.removed() &&
          member.identity().connection_epoch() == 1)
        return true;
    return false;
  }));
  const auto joined_before = server_->SnapshotStatusWork().subscribers_joined;
  watch.cancel();
  ASSERT_TRUE(watch.waitFinished());
  const auto join_deadline = Clock::now() + milliseconds(1500);
  while (Clock::now() < join_deadline &&
         server_->SnapshotStatusWork().subscribers_joined == joined_before)
    std::this_thread::sleep_for(milliseconds(1));
  ASSERT_GT(server_->SnapshotStatusWork().subscribers_joined, joined_before)
      << "the native status worker did not physically join";

  wire::SubscribeStatusRequest replacement;
  auto *next = replacement.mutable_selected()->add_members();
  *next->mutable_identity() = identity("other-0");
  next->add_channel_ids("state.test");
  Client::Watch new_watch(*client_, milliseconds(20000), replacement);
  ASSERT_TRUE(new_watch.wait([](const wire::SubscribeStatusResponse &frame) {
    for (const auto &member : frame.members())
      if (member.identity().robot_id() == "other-0" && member.messages_size())
        return true;
    return false;
  }));
  const auto last_join = server_->SnapshotStatusWork().subscribers_joined;
  new_watch.cancel();
  ASSERT_TRUE(new_watch.waitFinished());
  const auto end = Clock::now() + milliseconds(1500);
  while (Clock::now() < end && server_->SnapshotStatusWork().subscribers_joined == last_join)
    std::this_thread::sleep_for(milliseconds(10));
  EXPECT_GT(server_->SnapshotStatusWork().subscribers_joined, last_join)
      << "the native status owner did not join after cancellation";
}

TEST_F(ServerMembers, IdenticalViewsShareOneSourceAndLastReleaseJoinsIt) {
  startReady();
  wire::ApplyMembersRequest apply;
  *apply.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(apply).status.ok());
  wire::SubscribeStatusRequest selection;
  auto *item = selection.mutable_selected()->add_members();
  *item->mutable_identity() = identity("r1");
  item->add_channel_ids("state.test");
  const auto has_message = [](const wire::SubscribeStatusResponse &frame) {
    return frame.members_size() && frame.members(0).messages_size();
  };
  Client::Watch first(*client_, milliseconds(20000), selection);
  ASSERT_TRUE(first.wait(has_message));
  world_.holdTelemetry("r1", "state.test");
  const auto before_second = world_.telemetryCount("r1", "state.test");
  Client::Watch second(*client_, milliseconds(20000), selection);
  ASSERT_TRUE(second.wait(has_message));
  EXPECT_EQ(before_second, world_.telemetryCount("r1", "state.test"));
  first.cancel();
  ASSERT_TRUE(first.waitFinished());
  const auto joined_before = server_->SnapshotStatusWork().subscribers_joined;
  second.cancel();
  ASSERT_TRUE(second.waitFinished());
  const auto deadline = Clock::now() + milliseconds(1500);
  while (Clock::now() < deadline && server_->SnapshotStatusWork().subscribers != 0)
    std::this_thread::sleep_for(milliseconds(1));
  ASSERT_EQ(0u, server_->SnapshotStatusWork().subscribers);
  EXPECT_GT(server_->SnapshotStatusWork().subscribers_joined, joined_before);
  world_.releaseTelemetry("r1", "state.test");
  const auto before_idle = world_.telemetryCount("r1", "state.test");
  std::this_thread::sleep_for(milliseconds(600));
  EXPECT_EQ(before_idle, world_.telemetryCount("r1", "state.test"));
  Client::Watch reopened(*client_, milliseconds(20000), selection);
  ASSERT_TRUE(reopened.wait(has_message));
  EXPECT_GE(world_.telemetryCount("r1", "state.test"), before_idle + 1);
}

TEST_F(ServerMembers, StatusSelectionRejectsMissingDuplicateForeignAndOversizedRequests) {
  startReady();
  std::vector<wire::SubscribeStatusRequest> invalid(2);
  invalid[1].set_all(false);
  wire::SubscribeStatusRequest empty_selection;
  empty_selection.mutable_selected();
  invalid.push_back(empty_selection);
  wire::SubscribeStatusRequest foreign;
  auto *member = foreign.mutable_selected()->add_members();
  *member->mutable_identity() = identity("r1");
  member->mutable_identity()->set_target_id("foreign");
  member->add_channel_ids("state.test");
  invalid.push_back(foreign);
  wire::SubscribeStatusRequest duplicate_channel;
  member = duplicate_channel.mutable_selected()->add_members();
  *member->mutable_identity() = identity("r1");
  member->add_channel_ids("state.test");
  member->add_channel_ids("state.test");
  invalid.push_back(duplicate_channel);
  wire::SubscribeStatusRequest duplicate_member;
  *duplicate_member.mutable_selected()->add_members() = duplicate_channel.selected().members(0);
  duplicate_member.mutable_selected()->mutable_members(0)->mutable_channel_ids()->RemoveLast();
  *duplicate_member.mutable_selected()->add_members() = duplicate_member.selected().members(0);
  invalid.push_back(duplicate_member);
  wire::SubscribeStatusRequest oversized;
  member = oversized.mutable_selected()->add_members();
  *member->mutable_identity() = identity("r1");
  for (int i = 0; i != 33; ++i)
    member->add_channel_ids("channel-" + std::to_string(i));
  invalid.push_back(oversized);
  wire::SubscribeStatusRequest too_many_members;
  for (int i = 0; i != 257; ++i) {
    auto *selected = too_many_members.mutable_selected()->add_members();
    *selected->mutable_identity() = identity("r" + std::to_string(i));
    selected->add_channel_ids("state.test");
  }
  invalid.push_back(too_many_members);
  for (const auto &request : invalid) {
    Client::Watch watch(*client_, milliseconds(5000), request);
    ASSERT_TRUE(watch.waitFinished());
    EXPECT_EQ(grpc::StatusCode::INVALID_ARGUMENT, watch.status().error_code());
  }
}

TEST_F(ServerMembers, NativeStatusCapacityIsReleasedOnlyAfterTheWorkerJoins) {
  startReady();
  wire::ApplyMembersRequest apply;
  *apply.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(apply).status.ok());
  wire::SubscribeStatusRequest selection;
  auto *item = selection.mutable_selected()->add_members();
  *item->mutable_identity() = identity("r1");
  item->add_channel_ids("state.test");
  const auto has_message = [](const wire::SubscribeStatusResponse &frame) {
    return frame.members_size() && frame.members(0).messages_size();
  };
  std::vector<std::unique_ptr<Client::Watch>> admitted;
  for (int i = 0; i < 3; ++i) {
    admitted.emplace_back(new Client::Watch(*client_, milliseconds(20000), selection));
    ASSERT_TRUE(admitted.back()->wait(has_message));
  }
  EXPECT_EQ(3u, server_->SnapshotStatusWork().subscribers);
  Client::Watch refused(*client_, milliseconds(5000), selection);
  ASSERT_TRUE(refused.waitFinished());
  EXPECT_EQ(grpc::StatusCode::RESOURCE_EXHAUSTED, refused.status().error_code());
  EXPECT_EQ(3u, server_->SnapshotStatusWork().subscribers);
  const auto command_result = client_->execute(command("r1", "instant", "capacity-test"));
  ASSERT_TRUE(command_result.status.ok());
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, command_result.response.event().phase());

  const auto before = server_->SnapshotStatusWork().subscribers_joined;
  admitted.front()->cancel();
  ASSERT_TRUE(admitted.front()->waitFinished());
  const auto deadline = Clock::now() + milliseconds(1500);
  while (Clock::now() < deadline && server_->SnapshotStatusWork().subscribers_joined == before)
    std::this_thread::sleep_for(milliseconds(1));
  ASSERT_EQ(before + 1, server_->SnapshotStatusWork().subscribers_joined);
  EXPECT_EQ(2u, server_->SnapshotStatusWork().subscribers);
  Client::Watch replacement(*client_, milliseconds(20000), selection);
  ASSERT_TRUE(replacement.wait(has_message));
  EXPECT_EQ(3u, server_->SnapshotStatusWork().subscribers);
}

TEST_F(ServerMembers, SelectedStatusNeverSubstitutesAnotherEpoch) {
  startReady();
  wire::ApplyMembersRequest apply;
  *apply.add_members() = registration("r1", "/fleet/r1", "fake.profile", 2);
  ASSERT_TRUE(client_->apply(apply).status.ok());
  wire::SubscribeStatusRequest selection;
  auto *member = selection.mutable_selected()->add_members();
  *member->mutable_identity() = identity("r1", 1);
  member->add_channel_ids("state.test");
  Client::Watch watch(*client_, milliseconds(10000), selection);
  ASSERT_TRUE(watch.wait([](const wire::SubscribeStatusResponse &frame) {
    return frame.members_size() == 1 && frame.members(0).removed() &&
           frame.members(0).identity().connection_epoch() == 1;
  }));
  std::this_thread::sleep_for(milliseconds(300));
  for (const auto &frame : watch.frames())
    for (const auto &item : frame.members())
      EXPECT_EQ(1u, item.identity().connection_epoch());
}

using ServerCommands = Fixture;

TEST_F(ServerCommands, ExecuteRunsOnTheMembersQueueAndAnUncertainResultBlocksOnlyThatRobot) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  *request.add_members() = registration("r2", "/fleet/r2");
  ASSERT_TRUE(client_->apply(request).status.ok());

  auto done = client_->execute(command("r1", "instant", "w1"));
  ASSERT_TRUE(done.status.ok());
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, done.response.event().phase());
  EXPECT_EQ("w1", done.response.event().work_id());

  auto uncertain = client_->execute(command("r1", "uncertain", "w2"));
  EXPECT_EQ(operation::OPERATION_PHASE_UNCERTAIN, uncertain.response.event().phase());
  auto blocked = client_->execute(command("r1", "instant", "w3"));
  EXPECT_EQ(operation::OPERATION_PHASE_REJECTED, blocked.response.event().phase());
  EXPECT_EQ("previous-result-unknown", blocked.response.event().error().code());
  auto other = client_->execute(command("r2", "instant", "w4"));
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, other.response.event().phase());
}

// An unknown result latches the robot, but never the way out: the operator
// must still be able to release a held motion intent.
TEST_F(ServerCommands, AnUncertainResultDoesNotBlockASafetyOperation) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());

  EXPECT_EQ(operation::OPERATION_PHASE_UNCERTAIN,
            client_->execute(command("r1", "uncertain", "w1")).response.event().phase());
  EXPECT_EQ("previous-result-unknown",
            client_->execute(command("r1", "instant", "w2")).response.event().error().code());
  const auto released = client_->execute(command("r1", "release", "w3"));
  ASSERT_TRUE(released.status.ok());
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, released.response.event().phase());
  // The latch itself stays until the member is registered again.
  EXPECT_EQ("previous-result-unknown",
            client_->execute(command("r1", "instant", "w4")).response.event().error().code());
}

using ServerCoreSilence = Fixture;

// A member with no motion has no safety-publisher work, even with a view.
TEST_F(ServerCoreSilence, NoMotionDoesNotPollSafetyPublisher) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  *request.add_members() = registration("r2", "/fleet/r2");
  ASSERT_TRUE(client_->apply(request).status.ok());
  wire::SubscribeStatusRequest selection;
  auto *item = selection.mutable_selected()->add_members();
  *item->mutable_identity() = identity("r1");
  item->add_channel_ids("state.test");
  Client::Watch watch(*client_, milliseconds(20000), selection);
  ASSERT_TRUE(watch.wait([](const wire::SubscribeStatusResponse &frame) {
    return frame.members_size() > 0;
  }));
  const auto first = world_.silentCalls("r1");
  const auto second = world_.silentCalls("r2");
  std::this_thread::sleep_for(milliseconds(400));
  EXPECT_EQ(world_.silentCalls("r1"), first);
  EXPECT_EQ(world_.silentCalls("r2"), second);
}

// The silence clock is injectable: time moves without sleeping.
TEST(CoreSilenceClock, MeasuresFromTheEndOfTheLastStreamAndStreamsMayOverlap) {
  auto now = Clock::time_point{} + std::chrono::hours(1);
  CoreSilence silence([&now] { return now; });
  // Core never subscribed: silence runs from construction.
  now += std::chrono::seconds(2);
  EXPECT_EQ(milliseconds(2000), silence.SilentFor());

  silence.Opened();
  now += std::chrono::seconds(30);
  EXPECT_EQ(milliseconds(0), silence.SilentFor());
  // Renewal: the new stream opens before the old one ends; no silence between.
  silence.Opened();
  silence.Closed();
  now += std::chrono::seconds(5);
  EXPECT_EQ(milliseconds(0), silence.SilentFor());

  silence.Closed();
  EXPECT_EQ(milliseconds(0), silence.SilentFor());
  now += std::chrono::milliseconds(2999);
  EXPECT_EQ(milliseconds(2999), silence.SilentFor());
  now += std::chrono::milliseconds(1);
  EXPECT_EQ(milliseconds(3000), silence.SilentFor());
  silence.Opened();
  EXPECT_EQ(milliseconds(0), silence.SilentFor());
  // An unmatched Closed does not underflow the count.
  silence.Closed();
  silence.Closed();
  silence.Opened();
  EXPECT_EQ(milliseconds(0), silence.SilentFor());
}

TEST_F(ServerCommands, ExecuteNeedsTheActiveMemberOfThisEpoch) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  EXPECT_EQ("member-unavailable",
            client_->execute(command("nobody", "instant", "w1")).response.event().error().code());
  EXPECT_EQ("member-unavailable",
            client_->execute(command("r1", "instant", "w2", 5)).response.event().error().code());
  auto wrong_target = command("r1", "instant", "w3");
  wrong_target.mutable_identity()->set_target_id("foreign");
  EXPECT_EQ("member-unavailable", client_->execute(wrong_target).response.event().error().code());
  wire::RemoveMembersRequest wrong_remove;
  *wrong_remove.add_members() = wrong_target.identity();
  const auto refused = client_->remove(wrong_remove);
  ASSERT_TRUE(refused.status.ok());
  ASSERT_EQ(1, refused.response.members_size());
  EXPECT_EQ("member-target-conflict", refused.response.members(0).error().code());
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED,
            client_->execute(command("r1", "instant", "w4")).response.event().phase());
}

using ServerShutdown = Fixture;

TEST_F(ServerShutdown, DeadlineDoesNotPretendHeldNativeStopJoined) {
  auto timings = quickTimings();
  timings.member_drain = milliseconds(100);
  start(timings);
  ASSERT_TRUE(client_->describe(10000).response.ready());
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  world_.holdStop("r1");
  auto shutdown = std::async(std::launch::async, [this] {
    Client stopper(socket());
    return stopper.shutdown(milliseconds(800));
  });
  const bool entered = world_.waitStopEntered("r1", milliseconds(4000));
  EXPECT_TRUE(entered);
  const auto answer = shutdown.get();
  EXPECT_FALSE(answer.status.ok());
  EXPECT_FALSE(world_.contains("stop:r1"));
  EXPECT_FALSE(finished_.load());
  world_.releaseStop("r1");
  EXPECT_EQ(kExitOk, join());
  EXPECT_TRUE(world_.contains("stop:r1"));
}

TEST_F(ServerShutdown, CancelledNativeWorkJoinsBeforeShutdownReceipt) {
  auto timings = quickTimings();
  timings.command_drain = milliseconds(100);
  timings.member_drain = milliseconds(100);
  start(timings);
  ASSERT_TRUE(client_->describe(10000).response.ready());
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  world_.hold_cancelled_native = true;
  auto executing = std::async(std::launch::async, [this] {
    Client actor(socket());
    return actor.execute(command("r1", "slow", "shutdown-held-work"), milliseconds(10000));
  });
  for (int attempt = 0; attempt < 400 && !world_.contains("execute:r1:slow"); ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  EXPECT_TRUE(world_.contains("execute:r1:slow"));
  auto shutdown = std::async(std::launch::async, [this] {
    Client stopper(socket());
    return stopper.shutdown(milliseconds(10000));
  });
  EXPECT_EQ(std::future_status::timeout, shutdown.wait_for(milliseconds(800)))
      << "Shutdown answered before the native completion joined";
  EXPECT_FALSE(world_.contains("cancelled:shutdown-held-work"));
  EXPECT_FALSE(world_.contains("stop:r1"));
  EXPECT_EQ(1u, world_.native_cancel_attempts.load());
  world_.hold_cancelled_native = false;
  const auto stopped = shutdown.get();
  EXPECT_TRUE(stopped.status.ok()) << stopped.status.error_message();
  (void)executing.get();
  EXPECT_TRUE(world_.contains("cancelled:shutdown-held-work"));
  EXPECT_TRUE(world_.contains("stop:r1"));
  EXPECT_LT(world_.position("cancelled:shutdown-held-work"), world_.position("stop:r1"));
  EXPECT_EQ(kExitOk, join());
}

TEST_F(ServerShutdown, FailedNativeStopDoesNotProduceJoinedShutdown) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  {
    std::lock_guard<std::mutex> lock(world_.mutex);
    world_.failed_stops.insert("r1");
  }
  const auto answer = client_->shutdown();
  EXPECT_FALSE(answer.status.ok());
  EXPECT_EQ(grpc::StatusCode::UNKNOWN, answer.status.error_code());
  EXPECT_TRUE(world_.contains("stop-failed:r1"));
  EXPECT_EQ(kExitFailure, join());
}

TEST_F(ServerShutdown, TimedOutPublicationFlushDoesNotProduceJoinedShutdown) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  world_.fail_publication_flush = true;
  const auto answer = client_->shutdown();
  EXPECT_FALSE(answer.status.ok());
  EXPECT_EQ(grpc::StatusCode::UNKNOWN, answer.status.error_code());
  EXPECT_TRUE(world_.contains("stop:r1"));
  EXPECT_TRUE(world_.contains("flush-timeout"));
  EXPECT_EQ(kExitFailure, join());
}

TEST_F(ServerShutdown, DrainsInOrderThenAnswersAndExitsCleanly) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  *request.add_members() = registration("r2", "/fleet/r2");
  ASSERT_TRUE(client_->apply(request).status.ok());
  Client::Watch watch(*client_, milliseconds(20000), Client::Watch::broadSelection());

  // A command that is still in flight when the drain starts and never ends on
  // its own: the drain cancels it after its budget.
  Reply<wire::ExecuteResponse> in_flight;
  std::thread executor([&] {
    Client second(socket());
    in_flight = second.execute(command("r1", "slow", "slow-1"), milliseconds(10000));
  });
  for (int attempt = 0; attempt < 400 && !world_.contains("execute:r1:slow"); ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  ASSERT_TRUE(world_.contains("execute:r1:slow"));

  const auto answer = client_->shutdown();
  ASSERT_TRUE(answer.status.ok()) << answer.status.error_message();
  EXPECT_FALSE(answer.response.serving());
  executor.join();
  ASSERT_TRUE(in_flight.status.ok());
  EXPECT_EQ(operation::OPERATION_PHASE_CANCELLED, in_flight.response.event().phase());

  // By the time Shutdown answered, everything had been stopped, in order:
  // in-flight command cancelled before any robot stopped; every robot stopped.
  EXPECT_TRUE(world_.contains("cancelled:slow-1"));
  EXPECT_TRUE(world_.contains("stop:r1"));
  EXPECT_TRUE(world_.contains("stop:r2"));
  EXPECT_LT(world_.position("cancelled:slow-1"), world_.position("stop:r1"));
  EXPECT_EQ(0, join());
  EXPECT_TRUE(world_.contains("ros-shutdown"));
  EXPECT_LT(world_.position("stop:r2"), world_.position("ros-shutdown"));
  // One flush for all robots, after the last final command and before any
  // robot is destroyed.
  const auto lines = world_.snapshot();
  EXPECT_EQ(1, std::count(lines.begin(), lines.end(), "flush"));
  EXPECT_LT(world_.position("stop:r1"), world_.position("flush"));
  EXPECT_LT(world_.position("stop:r2"), world_.position("flush"));
  EXPECT_LT(world_.position("flush"), world_.position("destroy:r1"));
  EXPECT_LT(world_.position("flush"), world_.position("destroy:r2"));
  EXPECT_LT(world_.position("destroy:r2"), world_.position("ros-shutdown"));
  // The subscribers saw the members removed.
  EXPECT_TRUE(watch.waitFinished());
}

TEST_F(ServerShutdown, FleetRetirementUsesOneSchedulerBarrierBeforeNativeStops) {
  binding_->real_command_wait = true;
  startReady();
  wire::ApplyMembersRequest request;
  for (int i = 1; i <= 100; ++i) {
    const auto id = "r" + std::to_string(i);
    *request.add_members() = registration(id, "/fleet/" + id);
  }
  ASSERT_TRUE(client_->apply(request).status.ok());

  Reply<wire::ExecuteResponse> first, last;
  std::thread first_executor([&] {
    Client other(socket());
    first = other.execute(command("r1", "slow", "first"), milliseconds(20000));
  });
  std::thread last_executor([&] {
    Client other(socket());
    last = other.execute(command("r100", "slow", "last"), milliseconds(20000));
  });
  for (int attempt = 0; attempt < 400 &&
       (!world_.contains("execute:r1:slow") ||
        !world_.contains("execute:r100:slow")); ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  EXPECT_TRUE(world_.contains("execute:r1:slow"));
  EXPECT_TRUE(world_.contains("execute:r100:slow"));

  const auto answer = client_->shutdown(milliseconds(20000));
  first_executor.join();
  last_executor.join();
  ASSERT_TRUE(answer.status.ok()) << answer.status.error_message();
  EXPECT_FALSE(answer.response.serving());
  ASSERT_TRUE(first.status.ok()) << first.status.error_message();
  ASSERT_TRUE(last.status.ok()) << last.status.error_message();
  EXPECT_EQ(operation::OPERATION_PHASE_CANCELLED, first.response.event().phase());
  EXPECT_EQ(operation::OPERATION_PHASE_CANCELLED, last.response.event().phase());
  EXPECT_EQ(0, join());

  const auto lines = world_.snapshot();
  auto first_stop = std::find_if(lines.begin(), lines.end(), [](const auto &line) {
    return line.compare(0, 5, "stop:") == 0;
  });
  auto last_stop = std::find_if(lines.rbegin(), lines.rend(), [](const auto &line) {
    return line.compare(0, 5, "stop:") == 0;
  });
  ASSERT_NE(lines.end(), first_stop);
  ASSERT_NE(lines.rend(), last_stop);
  // Count the barrier's real zero-wait completion pump, before the final
  // flush. The scheduler's final I/O cleanup pump happens after that flush.
  const auto flush = lines.begin() + world_.position("flush");
  const auto fences = std::count(lines.begin(), flush, "fence-pump");
  RecordProperty("retirement_fence_pumps", fences);
  RecordProperty("idle_pumps_during_native_stops",
                 std::count(first_stop, last_stop.base(), "idle-pump"));
  EXPECT_EQ(1, fences);
  EXPECT_LT(world_.position("cancelled:first"), first_stop - lines.begin());
  EXPECT_LT(world_.position("cancelled:last"), first_stop - lines.begin());
  EXPECT_EQ(1, std::count(lines.begin(), lines.end(), "flush"));
  for (int i = 1; i <= 100; ++i) {
    const auto id = "r" + std::to_string(i);
    EXPECT_EQ(1, std::count(lines.begin(), lines.end(), "stop:" + id));
    EXPECT_EQ(1, std::count(lines.begin(), lines.end(), "destroy:" + id));
    EXPECT_LT(world_.position("stop:" + id), world_.position("flush"));
    EXPECT_LT(world_.position("flush"), world_.position("destroy:" + id));
    EXPECT_LT(world_.position("destroy:" + id), world_.position("ros-shutdown"));
  }
}

TEST_F(ServerShutdown, ACommandThatFinishesWithinTheBudgetIsNotCancelled) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  Reply<wire::ExecuteResponse> in_flight;
  std::thread executor([&] {
    Client second(socket());
    in_flight = second.execute(command("r1", "slow", "slow-1"));
  });
  for (int attempt = 0; attempt < 400 && !world_.contains("execute:r1:slow"); ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  std::thread finisher([&] {
    std::this_thread::sleep_for(milliseconds(100));
    std::lock_guard<std::mutex> lock(world_.mutex);
    auto pending = world_.pending.begin();
    ASSERT_NE(world_.pending.end(), pending);
    pending->second.complete(event(pending->second.request,
                                   operation::OPERATION_PHASE_SUCCEEDED));
    world_.pending.erase(pending);
  });
  ASSERT_TRUE(client_->shutdown().status.ok());
  executor.join();
  finisher.join();
  EXPECT_EQ(operation::OPERATION_PHASE_SUCCEEDED, in_flight.response.event().phase());
  EXPECT_FALSE(world_.contains("cancelled:slow-1"));
  EXPECT_TRUE(world_.contains("stop:r1"));
  EXPECT_EQ(0, join());
}

TEST_F(ServerShutdown, ASignalDrainsExactlyLikeTheRpc) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  server_->RequestStop();
  EXPECT_EQ(0, join());
  EXPECT_TRUE(world_.contains("stop:r1"));
  EXPECT_TRUE(world_.contains("ros-shutdown"));
}

TEST_F(ServerShutdown, AServerThatNeverGotReadyStillExitsOnRequest) {
  binding_->hold_connect = true;
  start();
  server_->RequestStop();
  binding_->releaseConnect();
  EXPECT_EQ(0, join());
}

TEST_F(ServerShutdown, NewWorkIsRefusedOnceTheDrainBegan) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  Reply<wire::ExecuteResponse> in_flight;
  std::thread executor([&] {
    Client second(socket());
    in_flight = second.execute(command("r1", "slow", "slow-1"));
  });
  for (int attempt = 0; attempt < 400 && !world_.contains("execute:r1:slow"); ++attempt)
    std::this_thread::sleep_for(milliseconds(5));
  std::thread shutdown([&] { client_->shutdown(); });
  // While the in-flight command holds the drain, the server already refuses.
  Reply<wire::ExecuteResponse> late;
  Reply<wire::ApplyMembersResponse> late_apply;
  for (int attempt = 0; attempt < 100; ++attempt) {
    Client third(socket());
    late = third.execute(command("r1", "instant", "late"));
    wire::ApplyMembersRequest more;
    *more.add_members() = registration("r9", "/fleet/r9");
    late_apply = third.apply(more);
    if (late_apply.status.error_code() == grpc::StatusCode::FAILED_PRECONDITION)
      break;
    std::this_thread::sleep_for(milliseconds(5));
  }
  EXPECT_EQ(grpc::StatusCode::FAILED_PRECONDITION, late_apply.status.error_code());
  if (late.status.ok()) {
    EXPECT_NE(operation::OPERATION_PHASE_SUCCEEDED, late.response.event().phase());
  }
  shutdown.join();
  executor.join();
  EXPECT_EQ(0, join());
}

using ServerMasterWatch = Fixture;

TEST_F(ServerMasterWatch, ExitsWhenTheMasterIsReplacedAfterStoppingTheRobots) {
  startReady();
  wire::ApplyMembersRequest request;
  *request.add_members() = registration("r1", "/fleet/r1");
  ASSERT_TRUE(client_->apply(request).status.ok());
  binding_->master_pid = 9999; // a restarted master has another pid
  EXPECT_EQ(kExitMasterChanged, join());
  EXPECT_TRUE(world_.contains("stop:r1"));
  EXPECT_TRUE(world_.contains("ros-shutdown"));
}

TEST_F(ServerMasterWatch, ExitsWhenTheMasterStaysUnreachable) {
  startReady();
  binding_->master_answers = false;
  const auto started = Clock::now();
  EXPECT_EQ(kExitMasterUnreachable, join());
  EXPECT_GE(Clock::now() - started, milliseconds(250));
}

TEST_F(ServerMasterWatch, ExitsWhenTheMasterRunIdChangesWithTheSamePid) {
  startReady();
  binding_->master_run_changed = true;
  EXPECT_EQ(kExitMasterChanged, join());
}

TEST_F(ServerMasterWatch, LearnsALateRunIdBeforeDetectingItsReplacement) {
  binding_->master_run_known = false;
  startReady();
  EXPECT_EQ("", facts(client_->describe().response)["ros_master_run_id"]);
  binding_->master_run_known = true;
  nlohmann::json described;
  for (int attempt = 0; attempt < 200; ++attempt) {
    described = facts(client_->describe().response);
    if (described["ros_master_run_id"] == "master-run-1")
      break;
    std::this_thread::sleep_for(milliseconds(5));
  }
  EXPECT_EQ("master-run-1", described["ros_master_run_id"]);
  EXPECT_TRUE(client_->health().response.serving());
  binding_->master_run_changed = true;
  EXPECT_EQ(kExitMasterChanged, join());
}

TEST_F(ServerMasterWatch, ABriefOutageIsNotAFailure) {
  startReady();
  binding_->master_answers = false;
  std::this_thread::sleep_for(milliseconds(120));
  binding_->master_answers = true;
  std::this_thread::sleep_for(milliseconds(400));
  EXPECT_TRUE(client_->health().response.serving());
  server_->RequestStop();
  EXPECT_EQ(0, join());
}

TEST_F(ServerMasterWatch, ExitsWhenRosShutsTheNodeDown) {
  startReady();
  binding_->running = false;
  EXPECT_EQ(kExitRosShutdown, join());
}

TEST_F(ServerMasterWatch, WaitsUntilTheMasterAcceptsTheNode) {
  binding_->master_calls_fail = true;
  start();
  const auto waiting = client_->describe(600);
  ASSERT_TRUE(waiting.status.ok());
  EXPECT_FALSE(waiting.response.ready());
  EXPECT_FALSE(finished_);
  binding_->master_calls_fail = false;
  EXPECT_TRUE(client_->describe(10000).response.ready());
  server_->RequestStop();
  EXPECT_EQ(kExitOk, join());
}

TEST_F(ServerMasterWatch, WaitsForALateMasterWithoutAStartupDeadline) {
  binding_->master_answers = false;
  start();
  const auto started = Clock::now();
  const auto waiting = client_->describe(600);
  ASSERT_TRUE(waiting.status.ok());
  EXPECT_FALSE(waiting.response.ready());
  EXPECT_GE(Clock::now() - started, milliseconds(550));
  EXPECT_EQ("waiting for ROS master binding", facts(waiting.response)["reason"]);
  EXPECT_FALSE(finished_);
  EXPECT_FALSE(world_.contains("connect"));
  EXPECT_FALSE(world_.contains("create:r1"));
  binding_->master_answers = true;
  EXPECT_TRUE(client_->describe(10000).response.ready());
  server_->RequestStop();
  EXPECT_EQ(kExitOk, join());
}

TEST_F(ServerMasterWatch, StopsCleanlyWhileWaitingForTheMaster) {
  binding_->master_answers = false;
  start();
  EXPECT_FALSE(client_->describe(0).response.ready());
  server_->RequestStop();
  EXPECT_EQ(kExitOk, join());
  EXPECT_TRUE(world_.contains("ros-shutdown"));
}

} // namespace
} // namespace xgc2_ros1_robot_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
