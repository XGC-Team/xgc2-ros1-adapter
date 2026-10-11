#include "xgc_scene/lidar/control.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <json/json.h>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>
#include <xgc2/xrpc/http.hpp>

namespace xgc_scene::lidar {
namespace {
using namespace xgc2::xrpc;
HttpResponse Reply(const Json::Value& value, int status = 200) {
    Json::StreamWriterBuilder writer;
    writer["indentation"] = "";
    return {status, {{"Content-Type", "application/json"}}, Json::writeString(writer, value)};
}
std::uint64_t Integer(const Json::Value& value, std::uint64_t high) {
    if ((value.type() != Json::intValue && value.type() != Json::uintValue) || !value.isUInt64() ||
        !value.asUInt64() || value.asUInt64() > high)
        throw std::invalid_argument("positive integer required");
    return value.asUInt64();
}
void Fields(const Json::Value& value, std::initializer_list<const char*> allowed) {
    if (!value.isObject())
        throw std::invalid_argument("configuration object required");
    for (const auto& name : value.getMemberNames()) {
        bool known = false;
        for (const auto* key : allowed)
            known = known || key == name;
        if (!known)
            throw std::invalid_argument("unknown lidar field: " + name);
    }
}
} // namespace

class Control::Impl : public std::enable_shared_from_this<Impl> {
public:
    struct Operation {
        std::string id, fingerprint, state, error;
        std::uint64_t revision = 0;
        bool enabled = false;
        Clock::time_point deadline, completed;
        HttpReply submit;
        std::vector<HttpReply> waiters;
    };
    std::string socket, target, instance = new_instance_id();
    std::function<void(std::function<void()>)> post;
    std::function<void(bool)> apply;
    std::mutex mutex;
    std::array<Operation, 128> operations;
    std::deque<std::size_t> pending;
    bool desired, applied, stopping = false, scheduled = false;
    std::uint64_t desired_revision = 1, applied_revision = 1;
    std::mutex lifecycle_mutex;
    std::condition_variable idle;
    bool applying = false, ready = true;
    std::atomic<bool> finish_io{false};
    struct ReadyWaiter {
        HttpReply reply;
        Clock::time_point until;
    };
    std::vector<ReadyWaiter> ready_waiters;
    std::unique_ptr<HttpServer> host;
    std::thread io;

    Impl(std::string p,
         std::string t,
         bool enabled,
         std::function<void(std::function<void()>)> executor,
         std::function<void(bool)> fn)
        : socket(std::move(p)), target(std::move(t)), post(std::move(executor)), apply(std::move(fn)),
          desired(enabled), applied(enabled) {
        if (target.empty() || !post || !apply)
            throw std::invalid_argument("lidar control identity and dispatcher required");
    }
    Json::Value Configuration() const {
        Json::Value value;
        value["desired"]["revision"] = Json::UInt64(desired_revision);
        value["desired"]["configuration"]["enabled"] = desired;
        value["applied"]["revision"] = Json::UInt64(applied_revision);
        value["applied"]["configuration"]["enabled"] = applied;
        value["persisted"] = Json::nullValue;
        return value;
    }
    Json::Value Receipt(const Operation& op) const {
        Json::Value value;
        value["id"] = op.id;
        value["kind"] = "source.configure";
        value["state"] = op.state;
        if (op.state == "succeeded") {
            value["result"]["revision"] = Json::UInt64(op.revision);
            value["result"]["configuration"]["enabled"] = op.enabled;
            value["result"]["persisted"] = Json::nullValue;
        } else if (op.state == "failed") {
            value["error"]["code"] = op.error;
            value["effects"]["applied"] = false;
        }
        return value;
    }
    Json::Value Describe() const {
        Json::Value value, ref;
        ref["target_id"] = target;
        ref["service"] = "xgc2.sensor.lidar";
        ref["api_version"] = "v1";
        ref["instance_id"] = instance;
        ref["profile"] = "http.v1";
        ref["endpoint"]["kind"] = "unix";
        ref["endpoint"]["address"] = socket;
        value["service_ref"] = ref;
        value["capabilities"].append("source.enabled.configure");
        value["limits"]["pending_operations"] = 16;
        value["limits"]["retained_operations"] = 128;
        value["limits"]["operation_timeout_ms"] = 60000;
        value["persistence"] = "none; launch configuration owns initial state";
        value["service"] = "xgc2.sensor.lidar";
        value["api_version"] = "v1";
        value["instance_id"] = instance;
        value["ready"] = ready && !stopping;
        value["facts"]["target_id"] = target;
        return value;
    }
    static bool ParseWait(const std::string& target, std::uint32_t& wait) {
        wait = 0;
        const auto query = target.find('?');
        if (query == std::string::npos)
            return true;
        const auto parameter = target.substr(query + 1);
        const std::string key = "wait_ready_ms=";
        if (parameter.compare(0, key.size(), key) != 0)
            return false;
        const auto digits = parameter.substr(key.size());
        if (digits.empty() || digits.size() > 5 ||
            digits.find_first_not_of("0123456789") != std::string::npos)
            return false;
        wait = static_cast<std::uint32_t>(std::stoul(digits));
        return wait <= 30000;
    }
    void ExpireReady() {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto waiter = ready_waiters.begin(); waiter != ready_waiters.end();) {
            if (ready || stopping || Clock::now() >= waiter->until || waiter->reply.cancelled()) {
                waiter->reply.complete(Reply(Describe()));
                waiter = ready_waiters.erase(waiter);
            } else {
                ++waiter;
            }
        }
    }
    void Finish(Operation& op, const std::string& failure) {
        op.state = failure.empty() ? "succeeded" : "failed";
        op.error = failure;
        op.completed = Clock::now();
        const auto response = Reply(Receipt(op));
        op.submit.complete(response);
        // Retained receipts must not retain HTTP admission or socket leases.
        op.submit = {};
        for (const auto& waiter : op.waiters)
            waiter.complete(response);
        op.waiters.clear();
    }
    void Handle(HttpRequest request, const HttpReply& reply) {
        try {
            std::unique_lock<std::mutex> lock(mutex);
            if (stopping) {
                reply.complete(http_error(503, "unavailable", "lidar source stopping"));
                return;
            }
            if (request.method == "GET" && request.target.substr(0, request.target.find('?')) == "/v1/describe") {
                std::uint32_t wait;
                if (!ParseWait(request.target, wait))
                    throw std::invalid_argument("the only query is wait_ready_ms between 0 and 30000");
                if (ready || wait == 0) {
                    reply.complete(Reply(Describe()));
                } else {
                    ready_waiters.push_back({reply, std::min(
                        Clock::now() + std::chrono::milliseconds(wait),
                        request.deadline - std::chrono::milliseconds(250))});
                }
                return;
            }
            if (request.method == "GET" &&
                (request.target == "/v1/status" || request.target == "/v1/health")) {
                auto value = Configuration();
                value["lifecycle"] = "ready";
                reply.complete(Reply(value));
                return;
            }
            constexpr const char* prefix = "/v1/operations/";
            if (request.target.rfind(prefix, 0) == 0) {
                std::string id = request.target.substr(std::char_traits<char>::length(prefix));
                const bool wait = request.method == "POST" && id.size() > 5 &&
                                  id.compare(id.size() - 5, 5, "/wait") == 0;
                if (wait)
                    id.resize(id.size() - 5);
                if ((!wait && request.method != "GET") || id.empty())
                    throw std::invalid_argument("unknown lidar operation route");
                for (auto& op : operations)
                    if (op.id == id) {
                        if (wait && op.state == "accepted") {
                            op.waiters.erase(
                                std::remove_if(op.waiters.begin(),
                                               op.waiters.end(),
                                               [](const HttpReply& r) { return r.cancelled(); }),
                                op.waiters.end());
                            if (op.waiters.size() == 4) {
                                reply.complete(http_error(
                                    429, "resource_exhausted", "lidar operation waiter limit"));
                                return;
                            }
                            op.waiters.push_back(reply);
                        } else
                            reply.complete(Reply(Receipt(op), op.state == "accepted" ? 202 : 200));
                        return;
                    }
                reply.complete(http_error(404, "not_found", "lidar operation unknown or expired"));
                return;
            }
            if (request.method != "POST" || request.target != "/v1/configure") {
                reply.complete(http_error(404, "not_found", "unknown lidar source route"));
                return;
            }
            Json::CharReaderBuilder builder;
            builder["rejectDupKeys"] = true;
            builder["failIfExtra"] = true;
            builder["allowComments"] = false;
            builder["stackLimit"] = 16;
            Json::Value value;
            std::string error;
            std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
            if (!reader->parse(
                    request.body.data(), request.body.data() + request.body.size(), &value, &error))
                throw std::invalid_argument("invalid lidar JSON");
            Fields(value, {"expected_revision", "configuration", "operation_timeout_ms"});
            Fields(value["configuration"], {"enabled"});
            if (!value["configuration"]["enabled"].isBool())
                throw std::invalid_argument("boolean enabled required");
            const auto expected = Integer(value["expected_revision"], 9007199254740991ULL);
            const auto budget = Integer(value["operation_timeout_ms"], 60000);
            for (auto& op : operations)
                if (op.id == request.request_id) {
                    if (op.fingerprint != request.body)
                        reply.complete(
                            http_error(409, "conflict", "lidar request ID payload conflict"));
                    else
                        reply.complete(Reply(Receipt(op), op.state == "accepted" ? 202 : 200));
                    return;
                }
            if (expected != desired_revision) {
                reply.complete(http_error(409, "conflict", "lidar desired revision conflict"));
                return;
            }
            if (pending.size() >= 16 || desired_revision == 9007199254740991ULL) {
                reply.complete(http_error(429, "resource_exhausted", "lidar operation capacity"));
                return;
            }
            std::size_t index = operations.size();
            Clock::time_point oldest = Clock::time_point::max();
            for (std::size_t i = 0; i < operations.size(); ++i) {
                if (operations[i].id.empty()) {
                    index = i;
                    break;
                }
                if (operations[i].state != "accepted" &&
                    operations[i].completed.time_since_epoch().count() <
                        oldest.time_since_epoch().count()) {
                    index = i;
                    oldest = operations[i].completed;
                }
            }
            if (index == operations.size()) {
                reply.complete(http_error(429, "resource_exhausted", "lidar receipt capacity"));
                return;
            }
            auto& op = operations[index];
            op = Operation{};
            op.id = request.request_id;
            op.fingerprint = request.body;
            op.state = "accepted";
            op.enabled = value["configuration"]["enabled"].asBool();
            op.revision = ++desired_revision;
            op.deadline = Clock::now() + std::chrono::milliseconds(budget);
            op.submit = reply;
            desired = op.enabled;
            pending.push_back(index);
            if (!scheduled) {
                scheduled = true;
                lock.unlock();
                const std::weak_ptr<Impl> weak = shared_from_this();
                try {
                    post([weak] {
                        if (const auto owner = weak.lock())
                            owner->ApplyPending();
                    });
                } catch (...) {
                    lock.lock();
                    for (const auto slot : pending)
                        Finish(operations[slot], "internal");
                    pending.clear();
                    scheduled = false;
                }
            }
        } catch (const std::invalid_argument& error) {
            reply.complete(http_error(400, "invalid_argument", error.what()));
        }
    }
    void ApplyPending() {
        for (;;) {
            std::size_t index;
            bool enabled;
            Clock::time_point deadline;
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (pending.empty() || stopping) {
                    scheduled = false;
                    return;
                }
                index = pending.front();
                pending.pop_front();
                enabled = operations[index].enabled;
                deadline = operations[index].deadline;
                applying = true;
            }
            std::string failure;
            if (Clock::now() >= deadline)
                failure = "deadline_exceeded";
            else
                try {
                    apply(enabled);
                } catch (...) {
                    failure = "internal";
                }
            std::lock_guard<std::mutex> lock(mutex);
            auto& op = operations[index];
            if (!stopping) {
                if (failure.empty()) {
                    applied = enabled;
                    applied_revision = op.revision;
                }
                Finish(op, failure);
            }
            applying = false;
            idle.notify_all();
        }
    }
};

Control::Control(std::string socket, std::string target, bool enabled,
                 std::function<void(std::function<void()>)> post,
                 std::function<void(bool)> apply)
    : impl_(std::make_shared<Impl>(std::move(socket), std::move(target), enabled,
                                   std::move(post), std::move(apply))) {}

Control::~Control() {
    Stop();
}

void Control::Start() {
    auto& s = *impl_;
    std::lock_guard<std::mutex> lifecycle(s.lifecycle_mutex);
    if (s.host || s.stopping)
        throw std::logic_error("lidar source host already started or stopped");
    HttpLimits limits;
    // Leave room for status, replay and wait requests alongside 16 submissions.
    limits.connections = 32;
    limits.inflight = 32;
    limits.request_bytes = 8192;
    limits.response_bytes = 8192;
    limits.request_timeout = std::chrono::seconds(60);
    s.host = std::make_unique<HttpServer>(
        UnixOptions{s.socket},
        [&s](HttpRequest r, const HttpReply& p) { s.Handle(std::move(r), p); },
        limits, HttpIdentity{s.instance, {"/v1/describe"}});
    s.io = std::thread([&s] {
        while (!s.finish_io.load()) {
            s.host->poll(std::chrono::milliseconds(25));
            s.ExpireReady();
        }
        s.host->drain();
    });
}

void Control::Stop() {
    auto& s = *impl_;
    std::lock_guard<std::mutex> lifecycle(s.lifecycle_mutex);
    {
        std::unique_lock<std::mutex> lock(s.mutex);
        if (s.stopping)
            return;
        s.stopping = true;
        for (auto& op : s.operations)
            if (op.state == "accepted")
                s.Finish(op, "unavailable");
        s.pending.clear();
        for (const auto& waiter : s.ready_waiters)
            waiter.reply.complete(Reply(s.Describe()));
        s.ready_waiters.clear();
        s.idle.wait(lock, [&s] { return !s.applying; });
    }
    s.finish_io.store(true);
    if (s.io.joinable())
        s.io.join();
    s.host.reset();
}

void Control::SetReady(bool ready) {
    auto& s = *impl_;
    std::lock_guard<std::mutex> lifecycle(s.lifecycle_mutex);
    {
        std::lock_guard<std::mutex> lock(s.mutex);
        s.ready = ready;
    }
    if (s.host)
        s.host->wake();
}

}  // namespace xgc_scene::lidar
