// ChannelTable resolves the generated contract once and indexes an array per
// ROS callback. These tests hold it to the string-keyed implementation it
// replaced. StringKeyedOracle below is that implementation's gate, envelope
// and source accounting, copied from RobotRuntime before the change; the only
// difference is that the wall-clock read inside the envelope is a parameter, so
// both implementations see the same value. Each replay drives the oracle and
// the table through the same call sequence that every PX4 ROS callback
// performs, records every outcome (including exception text) and compares the
// logs and the final per-channel state exactly.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>
#include <ros/time.h>

#include "xgc_px4_multirotor_ros1_adapter/channel_table.hpp"
#include "xgc_px4_multirotor_ros1_adapter/generated_contract.hpp"

namespace xgc_px4_multirotor_ros1_adapter {
namespace {

const char *const kExpectedChannelIds[kChannelCount] = {
    "state.localization.error",
    "state.pose",
    "state.mocap.pose",
    "state.vision.pose",
    "state.mocap.velocity",
    "state.mocap.speed",
    "state.mocap.acceleration",
    "state.velocity",
    "state.imu",
    "state.power",
    "state.controller",
    "state.health",
    "state.flight",
    "setpoint.local",
    "setpoint.attitude",
    "diagnostic.fcu-link",
    "diagnostic.offboard-input",
    "diagnostic.stream-health",
};

const char *const kRobotId = "uav1";

// ---------------------------------------------------------------------------
// The pre-change implementation (string-keyed maps, contract lookups per call).
// ---------------------------------------------------------------------------
class StringKeyedOracle {
public:
  StringKeyedOracle(const std::set<std::string> &enabled,
                    const std::set<std::string> &required)
      : profile_id_(contract::kProfileId), enabled_(enabled),
        required_(required) {}

  bool channelEnabled(std::size_t index) const {
    return enabled_.count(kExpectedChannelIds[index]) != 0;
  }
  bool channelRequired(std::size_t index) const {
    return required_.count(kExpectedChannelIds[index]) != 0;
  }

  void ensureSource(std::size_t index, double stale_after_seconds) {
    if (stale_after_seconds <= 0.0)
      throw std::logic_error("semantic source freshness must be positive");
    auto &source = sources_[kExpectedChannelIds[index]];
    source.stale_after_seconds = stale_after_seconds;
  }

  bool shouldEmit(std::size_t index, const ros::WallTime &now) {
    const std::string channel_id = kExpectedChannelIds[index];
    contract::ChannelMetadata metadata{};
    ++lookups;
    if (!contract::channelMetadata(profile_id_, channel_id, &metadata) ||
        metadata.output_rate_hz <= 0.0) {
      return false;
    }
    auto &last = last_output_[channel_id];
    if (!last.isZero() && now >= last &&
        (now - last).toSec() < (1.0 / metadata.output_rate_hz)) {
      return false;
    }
    last = now;
    return true;
  }

  xgc::robot::v1::RobotMessage
  makeEnvelope(std::size_t index, const ros::Time &source_stamp,
               const google::protobuf::Message &payload,
               std::int64_t observed_unix_nanos) {
    const std::string channel_id = kExpectedChannelIds[index];
    contract::ChannelMetadata channel{};
    ++lookups;
    if (!contract::channelMetadata(profile_id_, channel_id, &channel) ||
        channel.output_message_id == 0u) {
      throw std::logic_error(
          "channel output is absent from generated XGC2 contract metadata");
    }
    contract::MessageMetadata metadata{};
    ++lookups;
    if (!contract::messageMetadata(channel.output_message_id, &metadata)) {
      throw std::logic_error(
          "message ID is absent from generated XGC2 contract metadata");
    }
    xgc2_ros1_robot_adapter::MessageSchema schema;
    schema.message_id = channel.output_message_id;
    schema.type_name = metadata.type_name;
    schema.version = metadata.version;
    schema.fingerprint = metadata.fingerprint;

    xgc2_ros1_robot_adapter::RobotMessageContext context;
    context.robot_id = kRobotId;
    context.channel_id = channel_id;
    context.sequence = ++sequences_[channel_id];
    if (!source_stamp.isZero()) {
      context.has_source_time = true;
      context.source_time_nanos =
          static_cast<std::int64_t>(source_stamp.toNSec());
      context.source_clock_domain = ros::Time::isSimTime()
                                        ? xgc::v1::CLOCK_DOMAIN_SIMULATION
                                        : xgc::v1::CLOCK_DOMAIN_NATIVE;
    }
    context.observed_unix_nanos = observed_unix_nanos;

    xgc::robot::v1::RobotMessage envelope;
    std::string error;
    if (!xgc2_ros1_robot_adapter::BuildRobotMessage(context, schema, payload,
                                                    &envelope, &error)) {
      throw std::runtime_error("failed to build robot telemetry item: " +
                               error);
    }
    return envelope;
  }

  void recordSource(std::size_t index, const ros::WallTime &now) {
    const std::string channel_id = kExpectedChannelIds[index];
    auto source_it = sources_.find(channel_id);
    if (source_it == sources_.end())
      throw std::logic_error("semantic source was not installed: " +
                             channel_id);
    auto &source = source_it->second;
    if (source.window_started.isZero())
      source.window_started = now;
    if (!source.last_seen.isZero() && now > source.last_seen) {
      const double instantaneous_rate = 1.0 / (now - source.last_seen).toSec();
      source.source_rate_hz =
          source.source_rate_hz <= 0.0
              ? instantaneous_rate
              : 0.8 * source.source_rate_hz + 0.2 * instantaneous_rate;
    }
    source.last_seen = now;
    ++source.source_samples;
  }

  void recordOutput(std::size_t index) {
    const std::string channel_id = kExpectedChannelIds[index];
    auto source_it = sources_.find(channel_id);
    if (source_it == sources_.end())
      throw std::logic_error("semantic source was not installed: " +
                             channel_id);
    ++source_it->second.output_samples;
  }

  void countDropped(std::size_t index) {
    ++sources_[kExpectedChannelIds[index]].dropped_samples;
  }

  const std::map<std::string, SourceTracker> &sources() const {
    return sources_;
  }
  std::uint64_t lastOutputNanos(std::size_t index) const {
    const auto found = last_output_.find(kExpectedChannelIds[index]);
    return found == last_output_.end() ? 0u : found->second.toNSec();
  }
  std::uint64_t sequence(std::size_t index) const {
    const auto found = sequences_.find(kExpectedChannelIds[index]);
    return found == sequences_.end() ? 0u : found->second;
  }

  std::uint64_t lookups = 0u;

private:
  const std::string profile_id_;
  const std::set<std::string> enabled_;
  const std::set<std::string> required_;
  std::map<std::string, SourceTracker> sources_;
  std::map<std::string, ros::WallTime> last_output_;
  std::map<std::string, std::uint64_t> sequences_;
};

// ---------------------------------------------------------------------------
// The same operations through the resolved table.
// ---------------------------------------------------------------------------
class TableDriver {
public:
  TableDriver(const std::set<std::string> &enabled,
              const std::set<std::string> &required)
      : table_(contract::kProfileId, enabled, required) {}

  bool channelEnabled(std::size_t index) const {
    return table_[id(index)].enabled;
  }
  bool channelRequired(std::size_t index) const {
    return table_[id(index)].required;
  }
  void ensureSource(std::size_t index, double stale_after_seconds) {
    table_.ensureSource(&table_[id(index)], stale_after_seconds);
  }
  bool shouldEmit(std::size_t index, const ros::WallTime &now) {
    return table_.shouldEmit(&table_[id(index)], now);
  }
  xgc::robot::v1::RobotMessage
  makeEnvelope(std::size_t index, const ros::Time &source_stamp,
               const google::protobuf::Message &payload,
               std::int64_t observed_unix_nanos) {
    return table_.makeEnvelope(&table_[id(index)], kRobotId, source_stamp,
                               payload, observed_unix_nanos);
  }
  void recordSource(std::size_t index, const ros::WallTime &now) {
    table_.recordSource(&table_[id(index)], now);
  }
  void recordOutput(std::size_t index) {
    table_.recordOutput(&table_[id(index)]);
  }
  void countDropped(std::size_t index) {
    ++table_.droppedSource(&table_[id(index)]).dropped_samples;
  }

  const std::map<std::string, SourceTracker> &sources() const {
    return table_.sources();
  }
  std::uint64_t lastOutputNanos(std::size_t index) const {
    return table_[id(index)].last_output.toNSec();
  }
  std::uint64_t sequence(std::size_t index) const {
    return table_[id(index)].sequence;
  }
  std::uint64_t contractLookups() const { return table_.contractLookups(); }

private:
  static ChannelId id(std::size_t index) {
    return static_cast<ChannelId>(index);
  }
  ChannelTable table_;
};

// ---------------------------------------------------------------------------
// Replay harness.
// ---------------------------------------------------------------------------
struct Lcg {
  std::uint64_t state;
  std::uint32_t next() {
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<std::uint32_t>(state >> 33);
  }
};

ros::WallTime wallAt(std::int64_t nanos) {
  return ros::WallTime(static_cast<std::uint32_t>(nanos / 1000000000LL),
                       static_cast<std::uint32_t>(nanos % 1000000000LL));
}

ros::Time rosAt(std::int64_t nanos) {
  return ros::Time(static_cast<std::uint32_t>(nanos / 1000000000LL),
                   static_cast<std::uint32_t>(nanos % 1000000000LL));
}

// One semantic payload per channel, of exactly the type the generated contract
// declares for that channel's output message.
std::unique_ptr<google::protobuf::Message>
makePayload(std::size_t index) {
  contract::ChannelMetadata channel{};
  if (!contract::channelMetadata(contract::kProfileId,
                                 kExpectedChannelIds[index], &channel)) {
    return nullptr;
  }
  contract::MessageMetadata message{};
  if (!contract::messageMetadata(channel.output_message_id, &message))
    return nullptr;
  const google::protobuf::Descriptor *descriptor =
      google::protobuf::DescriptorPool::generated_pool()
          ->FindMessageTypeByName(message.type_name);
  if (descriptor == nullptr)
    return nullptr;
  const google::protobuf::Message *prototype =
      google::protobuf::MessageFactory::generated_factory()->GetPrototype(
          descriptor);
  if (prototype == nullptr)
    return nullptr;
  std::unique_ptr<google::protobuf::Message> payload(prototype->New());
  const google::protobuf::FieldDescriptor *field =
      descriptor->FindFieldByName("frame_id");
  if (field != nullptr && !field->is_repeated() &&
      field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_STRING) {
    payload->GetReflection()->SetString(payload.get(), field,
                                        "frame-" + std::to_string(index));
  }
  return payload;
}

struct ReplayCounters {
  std::uint64_t gate_checks = 0u;
  std::uint64_t envelope_calls = 0u;
  std::uint64_t emitted = 0u;
  std::uint64_t dropped = 0u;
  std::uint64_t source_failures = 0u;
};

template <typename Fn> std::string attempt(const std::string &label, Fn fn) {
  try {
    fn();
    return label + ":ok";
  } catch (const std::logic_error &error) {
    return label + ":logic_error:" + error.what();
  } catch (const std::exception &error) {
    return label + ":exception:" + error.what();
  }
}

// Every ROS callback performs: record the source, then either gate+envelope+
// output or count a dropped sample. `guarded` reproduces the callbacks that
// record a source only when it is required.
template <typename Driver>
std::vector<std::string>
replay(Driver *driver, std::uint64_t seed, int steps, bool guarded,
       const std::vector<std::unique_ptr<google::protobuf::Message>> &payloads,
       ReplayCounters *counters) {
  static const std::int64_t kDeltas[] = {
      0LL,          1000000LL,   9999999LL,   10000000LL,  50000000LL,
      99999999LL,   100000000LL, 100000001LL, 250000000LL, 1000000000LL};
  const std::int64_t kStart = 1000LL * 1000000000LL;
  Lcg rng{seed};
  std::int64_t now_ns = kStart;
  std::vector<std::string> log;
  log.reserve(static_cast<std::size_t>(steps) * 4u);
  for (int step = 0; step < steps; ++step) {
    const std::uint32_t draw = rng.next();
    const std::size_t channel = draw % kChannelCount;
    if ((draw >> 16) % 37u == 0u) {
      now_ns -= 400000000LL; // wall-clock rollback behind the last output
    } else {
      now_ns += kDeltas[(draw >> 8) % 10u];
    }
    if (now_ns < kStart - 5000000000LL)
      now_ns = kStart - 5000000000LL;
    const ros::WallTime now = wallAt(now_ns);
    const ros::Time stamp =
        ((draw >> 24) % 5u == 0u) ? ros::Time() : rosAt(now_ns - 1000LL);
    const std::int64_t observed = now_ns + 1;

    if (!guarded || driver->channelRequired(channel)) {
      const std::string result =
          attempt("record", [&] { driver->recordSource(channel, now); });
      if (result.find("logic_error") != std::string::npos)
        ++counters->source_failures;
      log.push_back(result);
    }
    if (!driver->channelEnabled(channel))
      continue;
    ++counters->gate_checks;
    if (driver->shouldEmit(channel, now)) {
      log.push_back("gate:1");
      ++counters->envelope_calls;
      std::string serialized;
      log.push_back(attempt("envelope", [&] {
        serialized = driver->makeEnvelope(channel, stamp, *payloads[channel],
                                          observed)
                         .SerializeAsString();
      }));
      log.push_back(serialized);
      log.push_back(attempt("output", [&] { driver->recordOutput(channel); }));
      ++counters->emitted;
    } else {
      log.push_back("gate:0");
      driver->countDropped(channel);
      ++counters->dropped;
    }
  }
  return log;
}

std::string describeState(const std::map<std::string, SourceTracker> &sources) {
  std::ostringstream out;
  out << std::hexfloat;
  for (const auto &entry : sources) {
    const SourceTracker &s = entry.second;
    out << entry.first << '|' << s.last_seen.toNSec() << '|'
        << s.window_started.toNSec() << '|' << s.source_samples << '|'
        << s.output_samples << '|' << s.dropped_samples << '|'
        << s.source_rate_hz << '|' << s.output_rate_hz << '|'
        << s.stale_after_seconds << '\n';
  }
  return out.str();
}

std::set<std::string> everyChannel() {
  return std::set<std::string>(kExpectedChannelIds,
                               kExpectedChannelIds + kChannelCount);
}

std::set<std::string> everyOtherChannel(std::size_t first) {
  std::set<std::string> subset;
  for (std::size_t index = first; index < kChannelCount; index += 2u)
    subset.insert(kExpectedChannelIds[index]);
  return subset;
}

class ReplayFixture : public ::testing::Test {
protected:
  void SetUp() override {
    payloads.resize(kChannelCount);
    for (std::size_t index = 0u; index < kChannelCount; ++index) {
      payloads[index] = makePayload(index);
      ASSERT_TRUE(payloads[index] != nullptr)
          << "no generated payload type for " << kExpectedChannelIds[index];
    }
  }

  struct Outcome {
    std::vector<std::string> log;
    std::string state;
    std::vector<std::uint64_t> last_output;
    std::vector<std::uint64_t> sequence;
    ReplayCounters counters;
  };

  template <typename Driver>
  Outcome run(const std::set<std::string> &enabled,
              const std::set<std::string> &required, std::uint64_t seed,
              bool guarded) {
    Driver driver(enabled, required);
    // Installation: a source exists exactly for the required channels.
    for (std::size_t index = 0u; index < kChannelCount; ++index) {
      if (driver.channelRequired(index))
        driver.ensureSource(index, 0.25 * static_cast<double>(index + 1u));
    }
    Outcome outcome;
    outcome.log = replay(&driver, seed, 30000, guarded, payloads,
                         &outcome.counters);
    outcome.state = describeState(driver.sources());
    for (std::size_t index = 0u; index < kChannelCount; ++index) {
      outcome.last_output.push_back(driver.lastOutputNanos(index));
      outcome.sequence.push_back(driver.sequence(index));
    }
    return outcome;
  }

  void expectSame(const std::set<std::string> &enabled,
                  const std::set<std::string> &required, std::uint64_t seed,
                  bool guarded, bool expect_traffic = true) {
    const Outcome expected =
        run<StringKeyedOracle>(enabled, required, seed, guarded);
    const Outcome actual = run<TableDriver>(enabled, required, seed, guarded);
    ASSERT_EQ(expected.log.size(), actual.log.size());
    for (std::size_t index = 0u; index < expected.log.size(); ++index) {
      ASSERT_EQ(expected.log[index], actual.log[index])
          << "first divergence at log entry " << index;
    }
    EXPECT_EQ(expected.state, actual.state);
    EXPECT_EQ(expected.last_output, actual.last_output);
    EXPECT_EQ(expected.sequence, actual.sequence);
    // The replay must have exercised the interesting paths, otherwise the
    // equality above proves little.
    if (expect_traffic) {
      EXPECT_GT(actual.counters.emitted, 100u);
      EXPECT_GT(actual.counters.dropped, 100u);
    } else {
      EXPECT_EQ(0u, actual.counters.emitted);
    }
  }

  std::vector<std::unique_ptr<google::protobuf::Message>> payloads;
};

TEST(ChannelTableContract, ChannelIdsFollowTheDeclaredOrder) {
  for (std::size_t index = 0u; index < kChannelCount; ++index) {
    EXPECT_STREQ(kExpectedChannelIds[index],
                 channelIdName(static_cast<ChannelId>(index)))
        << "index " << index;
  }
  EXPECT_EQ(18u, kChannelCount);
}

TEST(ChannelTableContract, ResolvesEveryChannelFromTheGeneratedProfile) {
  const ChannelTable table(contract::kProfileId, everyChannel(),
                           everyOtherChannel(0u));
  for (std::size_t index = 0u; index < kChannelCount; ++index) {
    const ChannelTable::Channel &channel = table[static_cast<ChannelId>(index)];
    contract::ChannelMetadata metadata{};
    ASSERT_TRUE(contract::channelMetadata(
        contract::kProfileId, kExpectedChannelIds[index], &metadata))
        << kExpectedChannelIds[index];
    EXPECT_EQ(kExpectedChannelIds[index], channel.id);
    EXPECT_TRUE(channel.enabled);
    EXPECT_EQ(index % 2u == 0u, channel.required);
    EXPECT_TRUE(channel.metadata_found);
    EXPECT_EQ(metadata.output_rate_hz, channel.output_rate_hz);
    EXPECT_EQ(metadata.stale_after_millis, channel.stale_after_millis);
    ASSERT_GT(metadata.output_rate_hz, 0.0) << kExpectedChannelIds[index];
    EXPECT_EQ(1.0 / metadata.output_rate_hz, channel.emit_interval_seconds);
    EXPECT_EQ(metadata.observes_count, channel.observes.size());
    ASSERT_EQ(ChannelTable::Channel::Envelope::kReady, channel.envelope);
    contract::MessageMetadata message{};
    ASSERT_TRUE(contract::messageMetadata(metadata.output_message_id, &message));
    EXPECT_EQ(metadata.output_message_id, channel.schema.message_id);
    EXPECT_EQ(message.type_name, channel.schema.type_name);
    EXPECT_EQ(message.version, channel.schema.version);
    EXPECT_EQ(message.fingerprint, channel.schema.fingerprint);
    EXPECT_TRUE(channel.source == nullptr);
  }
}

TEST(ChannelTableContract, UnknownProfileResolvesNothingAndNeverEmits) {
  ChannelTable table("no.such.profile", everyChannel(), everyChannel());
  ChannelTable::Channel &channel = table[ChannelId::kPose];
  EXPECT_FALSE(channel.metadata_found);
  EXPECT_FALSE(table.shouldEmit(&channel, wallAt(5000000000LL)));
  EXPECT_FALSE(table.shouldEmit(&channel, wallAt(9000000000LL)));
  const std::unique_ptr<google::protobuf::Message> payload = makePayload(1u);
  ASSERT_TRUE(payload != nullptr);
  try {
    table.makeEnvelope(&channel, kRobotId, ros::Time(), *payload, 1);
    FAIL() << "an unresolved channel produced an envelope";
  } catch (const std::logic_error &error) {
    EXPECT_STREQ(
        "channel output is absent from generated XGC2 contract metadata",
        error.what());
  }
  EXPECT_EQ(0u, channel.sequence);
}

TEST(ChannelTableGate, FirstSampleRollbackAndIntervalFollowTheOutputRate) {
  ChannelTable table(contract::kProfileId, everyChannel(), everyChannel());
  ChannelTable::Channel &channel = table[ChannelId::kPose];
  const std::int64_t interval_ns = static_cast<std::int64_t>(
      channel.emit_interval_seconds * 1e9);
  const std::int64_t t0 = 50LL * 1000000000LL;
  EXPECT_TRUE(table.shouldEmit(&channel, wallAt(t0))); // first sample
  EXPECT_FALSE(table.shouldEmit(&channel, wallAt(t0 + interval_ns / 2)));
  EXPECT_FALSE(table.shouldEmit(&channel, wallAt(t0)));
  EXPECT_TRUE(table.shouldEmit(&channel, wallAt(t0 + 2 * interval_ns)));
  // A wall clock that moved backwards restarts the interval.
  EXPECT_TRUE(table.shouldEmit(&channel, wallAt(t0 + interval_ns)));
  EXPECT_FALSE(table.shouldEmit(&channel, wallAt(t0 + interval_ns + 1000)));
  EXPECT_EQ(wallAt(t0 + interval_ns).toNSec(), channel.last_output.toNSec());
}

TEST(ChannelTableSources, FreshnessMustBePositiveAndDroppedSamplesCreateTrackers) {
  ChannelTable table(contract::kProfileId, everyChannel(), everyOtherChannel(0u));
  ChannelTable::Channel &pose = table[ChannelId::kPose];
  for (const double bad : {0.0, -1.0}) {
    try {
      table.ensureSource(&pose, bad);
      FAIL() << "non-positive freshness was accepted";
    } catch (const std::logic_error &error) {
      EXPECT_STREQ("semantic source freshness must be positive", error.what());
    }
  }
  EXPECT_TRUE(pose.source == nullptr);
  EXPECT_TRUE(table.sources().empty());

  // Not installed: recording is a logic error naming the channel.
  try {
    table.recordSource(&pose, wallAt(1000000000LL));
    FAIL() << "an uninstalled source was recorded";
  } catch (const std::logic_error &error) {
    EXPECT_STREQ("semantic source was not installed: state.pose", error.what());
  }
  try {
    table.recordOutput(&pose);
    FAIL() << "an uninstalled source counted an output";
  } catch (const std::logic_error &error) {
    EXPECT_STREQ("semantic source was not installed: state.pose", error.what());
  }

  // A dropped sample installs a default tracker, as the map did.
  ++table.droppedSource(&pose).dropped_samples;
  ASSERT_TRUE(pose.source != nullptr);
  ASSERT_EQ(1u, table.sources().size());
  EXPECT_EQ(&table.sources().at("state.pose"), pose.source);
  EXPECT_EQ(1u, pose.source->dropped_samples);
  EXPECT_EQ(1.0, pose.source->stale_after_seconds);
  // From then on the source exists for recording, again as before.
  table.recordSource(&pose, wallAt(1000000000LL));
  EXPECT_EQ(1u, pose.source->source_samples);
  // Installing later keeps the same node and updates its freshness bound.
  table.ensureSource(&pose, 0.75);
  EXPECT_EQ(&table.sources().at("state.pose"), pose.source);
  EXPECT_EQ(0.75, pose.source->stale_after_seconds);
}

TEST_F(ReplayFixture, MatchesTheStringKeyedImplementationWhenEverythingIsOn) {
  expectSame(everyChannel(), everyChannel(), 1u, false);
  expectSame(everyChannel(), everyChannel(), 7u, true);
}

TEST_F(ReplayFixture, MatchesWhenOnlyAlternateChannelsAreRequired) {
  // Enabled-but-not-required channels hit the uninstalled-source exception on
  // record and the lazy tracker on drop; both must match exactly.
  expectSame(everyChannel(), everyOtherChannel(0u), 11u, false);
  expectSame(everyChannel(), everyOtherChannel(1u), 13u, false);
  expectSame(everyChannel(), everyOtherChannel(0u), 17u, true);
}

TEST_F(ReplayFixture, MatchesWhenChannelsAreDisabled) {
  expectSame(everyOtherChannel(0u), everyChannel(), 19u, false);
  expectSame(everyOtherChannel(1u), everyOtherChannel(1u), 23u, true);
  expectSame(std::set<std::string>(), everyChannel(), 29u, false, false);
}

TEST_F(ReplayFixture, ResolvesTheContractOnceInsteadOfOncePerCallback) {
  const std::set<std::string> all = everyChannel();
  TableDriver driver(all, all);
  const std::uint64_t at_construction = driver.contractLookups();
  // One channel lookup and one message lookup per channel, nothing else.
  EXPECT_EQ(2u * kChannelCount, at_construction);
  for (std::size_t index = 0u; index < kChannelCount; ++index)
    driver.ensureSource(index, 1.0);
  ReplayCounters counters;
  replay(&driver, 31u, 30000, false, payloads, &counters);
  ASSERT_GT(counters.gate_checks, 1000u);
  EXPECT_EQ(at_construction, driver.contractLookups());

  // The pre-change implementation consulted the contract on every gate check
  // and twice on every envelope.
  StringKeyedOracle counting(all, all);
  for (std::size_t index = 0u; index < kChannelCount; ++index)
    counting.ensureSource(index, 1.0);
  ReplayCounters old_counters;
  replay(&counting, 31u, 30000, false, payloads, &old_counters);
  ASSERT_GT(old_counters.gate_checks, 1000u);
  EXPECT_EQ(old_counters.gate_checks + 2u * old_counters.emitted,
            counting.lookups);
  EXPECT_GT(counting.lookups, 100u * at_construction);
}

} // namespace
} // namespace xgc_px4_multirotor_ros1_adapter
