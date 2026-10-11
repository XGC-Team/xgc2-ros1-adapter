#include "xgc_scene/store.hpp"

#include "xgc_scene/document.hpp"
#include "xgc_scene/json_io.hpp"
#include "xgc_scene/motion.hpp"
#include "xgc_scene/yaml_io.hpp"
#include "scalars.hpp"

#include <openssl/rand.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <regex>
#include <sys/stat.h>
#include <system_error>
#include <unistd.h>

namespace xgc_scene {
namespace fs = std::filesystem;
namespace detail {
void replaceStoreFile(const fs::path& temporary, const fs::path& target);
}
namespace {

constexpr char kExternalEdit[] = "Scene YAML changed outside the editor; reload YAML before editing";

// JSON equality intentionally follows dict equality. Text caches additionally
// distinguish insertion order, numeric types and the sign of floating zero.
bool sameTextValue(const Json& a, const Json& b) {
  if (a.type() != b.type() || a.size() != b.size()) return false;
  if (a.is_number_float()) {
    const auto x = a.get<double>(), y = b.get<double>();
    return x == y && (x != 0 || std::signbit(x) == std::signbit(y));
  }
  if (a.is_structured()) {
    auto left = a.begin(), right = b.begin();
    for (; left != a.end(); ++left, ++right) {
      if (a.is_object() && left.key() != right.key()) return false;
      if (!sameTextValue(left.value(), right.value())) return false;
    }
    return true;
  }
  return a == b;
}

std::string itemKey(const Json& value, std::size_t index) {
  if (value.is_object() && value.contains("id") && value.at("id").is_string())
    return value.at("id").get<std::string>();
  return "#" + std::to_string(index);
}

std::string ioError(int code, const fs::path& path, const std::optional<fs::path>& target = {}) {
  std::string message = "[Errno " + std::to_string(code) + "] " + std::generic_category().message(code) +
                        ": " + detail::pythonRepr(Json(path.string()));
  if (target) message += " -> " + detail::pythonRepr(Json(target->string()));
  return message;
}

std::optional<fs::path> resolved(const std::optional<fs::path>& path) {
  if (!path || path->empty()) return {};
  return fs::weakly_canonical(fs::absolute(*path));
}

std::pair<Json, std::string> readDocument(const fs::path& path) {
  try { return load(path); }
  catch (const std::system_error& error) { throw SceneError(ioError(error.code().value(), path)); }
}

std::optional<std::string> currentDigest(const fs::path& path) {
  try {
    if (!fs::exists(path)) return {};
    return fileDigest(path);
  } catch (const std::system_error& error) { throw SceneError(ioError(error.code().value(), path)); }
}

Json sortedFields(const Json& value) {
  if (value.is_array()) {
    Json result = Json::array();
    for (const auto& item : value) result.push_back(sortedFields(item));
    return result;
  }
  if (!value.is_object()) return value;
  std::vector<std::string> keys;
  for (auto it = value.begin(); it != value.end(); ++it) keys.push_back(it.key());
  std::sort(keys.begin(), keys.end());
  Json result = Json::object();
  for (const auto& key : keys) result[key] = sortedFields(value.at(key));
  return result;
}

std::string freshEpoch() {
  std::array<unsigned char, 16> bytes{};
  if (RAND_bytes(bytes.data(), bytes.size()) != 1) throw SceneError("Cannot create scene epoch");
  bytes[6] = (bytes[6] & 0x0f) | 0x40;
  bytes[8] = (bytes[8] & 0x3f) | 0x80;
  constexpr char hex[] = "0123456789abcdef";
  std::string result;
  for (const auto byte : bytes) { result += hex[byte >> 4]; result += hex[byte & 15]; }
  return result;
}

class HookGuard {
 public:
  explicit HookGuard(bool& flag) : flag_(flag), previous_(flag) { flag_ = true; }
  ~HookGuard() { flag_ = previous_; }
 private:
  bool& flag_;
  bool previous_;
};

class SaveFile {
 public:
  explicit SaveFile(const fs::path& target) {
    auto pattern = (target.parent_path() / ("." + target.filename().string() + ".XXXXXX.tmp")).string();
    std::vector<char> name(pattern.begin(), pattern.end());
    name.push_back('\0');
    descriptor = ::mkstemps(name.data(), 4);
    if (descriptor < 0) throw SceneError(ioError(errno, target.parent_path()));
    path = name.data();
  }
  ~SaveFile() {
    if (descriptor >= 0) ::close(descriptor);
    if (!path.empty()) ::unlink(path.c_str());
  }
  void close() {
    const auto handle = descriptor;
    descriptor = -1;
    if (::close(handle) != 0) throw SceneError(ioError(errno, path));
  }
  int descriptor = -1;
  fs::path path;
};

}  // namespace

std::string DocumentYaml::dump(const Json& value) {
  const auto& obstacles = value.at("obstacles");
  if (obstacles.empty() || std::prev(value.end()).key() != "obstacles") {
    items_.clear();
    emitted_obstacles_ += obstacles.size();
    return dumpYaml(value);
  }
  decltype(items_) items;
  std::string text;
  std::size_t index = 0;
  for (const auto& obstacle : obstacles) {
    const auto key = itemKey(obstacle, index++);
    const auto cached = items_.find(key);
    std::shared_ptr<const Item> item;
    if (cached != items_.end() && sameTextValue(cached->second->value, obstacle)) item = cached->second;
    else {
      item = std::make_shared<Item>(Item{obstacle, dumpYaml(Json::array({obstacle}))});
      ++emitted_obstacles_;
    }
    text += item->text;
    items[key] = std::move(item);
  }
  items_ = std::move(items);
  Json header = Json::object();
  for (auto it = value.begin(); it != value.end(); ++it) if (it.key() != "obstacles") header[it.key()] = it.value();
  return dumpYaml(header) + "obstacles:\n" + text;
}

std::string EnvelopeJson::documentLocked(const Json& value) {
  if (document_ && sameTextValue(*document_, value)) return document_text_;
  std::string text;
  if (!value.is_object() || !value.contains("obstacles") || !value.at("obstacles").is_array()) text = dumpJson(value);
  else {
    decltype(obstacles_) items;
    std::string listed = "[";
    std::size_t index = 0;
    for (const auto& obstacle : value.at("obstacles")) {
      const auto key = itemKey(obstacle, index);
      if (index++) listed += ", ";
      const auto cached = obstacles_.find(key);
      std::shared_ptr<const Item> item;
      if (cached != obstacles_.end() && sameTextValue(cached->second->value, obstacle)) item = cached->second;
      else {
        item = std::make_shared<Item>(Item{obstacle, dumpJson(obstacle)});
        ++emitted_obstacles_;
      }
      listed += item->text;
      items[key] = std::move(item);
    }
    listed += "]";
    text = "{";
    for (auto it = value.begin(); it != value.end(); ++it) {
      if (it != value.begin()) text += ", ";
      text += dumpJson(Json(it.key())) + ": " + (it.key() == "obstacles" ? listed : dumpJson(it.value()));
    }
    text += "}";
    obstacles_ = std::move(items);
  }
  document_ = value;
  document_text_ = text;
  return text;
}

std::string EnvelopeJson::document(const Json& value) {
  std::lock_guard<std::mutex> lock(mutex_);
  return documentLocked(value);
}

std::size_t EnvelopeJson::obstacleEmissions() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return emitted_obstacles_;
}

std::string EnvelopeJson::dumps(const Json& value) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!value.is_object() || !value.contains("document") || !value.at("document").is_object()) return dumpJson(value);
  std::string text = "{";
  for (auto it = value.begin(); it != value.end(); ++it) {
    if (it != value.begin()) text += ", ";
    text += dumpJson(Json(it.key())) + ": " + (it.key() == "document" ? documentLocked(it.value()) : dumpJson(it.value()));
  }
  return text + "}";
}

std::string envelopeJson(const Json& value) {
  thread_local EnvelopeJson encoder;
  return encoder.dumps(value);
}

SceneStore::SceneStore(const Json& initial, std::optional<fs::path> source, std::optional<fs::path> save_root,
                       Apply apply_hook, Clock clock, std::optional<std::string> source_digest,
                       bool frozen, std::optional<fs::path> working_file, Motion motion_hook)
    : frozen_(frozen), working_file_(resolved(working_file)), epoch_(freshEpoch()), source_(resolved(source)),
      save_root_(resolved(save_root)), apply_(std::move(apply_hook)), motion_(std::move(motion_hook)), clock_(std::move(clock)) {
  Json input = initial;
  if (working_file_ && fs::exists(*working_file_)) {
    source_ = working_file_;
    auto loaded = readDocument(*source_);
    input = std::move(loaded.first);
    source_digest = std::move(loaded.second);
  }
  document_ = std::make_shared<const Json>(xgc_scene::document(input));
  obstacle_validations_ = document_->at("obstacles").size();
  saved_document_ = document_;
  const auto& target = working_file_ ? working_file_ : source_;
  if (!save_root_ && target) save_root_ = target->parent_path();
  if (source_ && fs::exists(*source_)) {
    file_digests_[source_->string()] = source_digest && !source_digest->empty() ? *source_digest : *currentDigest(*source_);
  }
  if (!apply_) apply_ = [](const Json&, const std::string&, Revision) {};
  if (!motion_) motion_ = [](const std::string&, const std::string&, Revision) {};
  if (!clock_) clock_ = [] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  };
  started_at_ = nowLocked();
  {
    HookGuard hook(in_hook_);
    apply_(*document_, epoch_, revision_);
  }
  applied_revision_ = revision_;
  application_known_ = true;
}

double SceneStore::nowLocked() const {
  HookGuard hook(in_hook_);
  return clock_();
}

double SceneStore::sceneTimeLocked() const {
  return elapsed_ + (playing_ ? std::max(0.0, nowLocked() - started_at_) : 0.0);
}

double SceneStore::sceneTime() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return sceneTimeLocked();
}

Json SceneStore::statusLocked() const {
  return Json{{"epoch", epoch_}, {"revision", revision_}, {"savedRevision", saved_revision_},
              {"dirty", document_ != saved_document_ && *document_ != *saved_document_},
              {"playing", playing_}, {"frozen", frozen_},
              {"configuration", Json{{"desired", Json{{"epoch", epoch_}, {"revision", revision_}}},
                                     {"applied", Json{{"epoch", epoch_}, {"revision", applied_revision_}, {"known", application_known_}}},
                                     {"persisted", Json{{"epoch", epoch_}, {"revision", saved_revision_}}}}}};
}

Json SceneStore::status() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return statusLocked();
}

Json SceneStore::envelopeLocked() const {
  auto result = statusLocked();
  result["sceneTime"] = sceneTimeLocked();
  result["document"] = *document_;
  return result;
}

Json SceneStore::envelope() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return envelopeLocked();
}

Json SceneStore::states() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const double elapsed = sceneTimeLocked();
  Json result = Json::array();
  for (const auto& item : document_->at("obstacles")) result.push_back(state(item, elapsed, playing_));
  return result;
}

Json SceneStore::resultLocked(bool success, const std::string& error) const {
  auto result = envelopeLocked();
  result["success"] = success;
  if (!success) result["error"] = error;
  return result;
}

Json SceneStore::receiptValue(const Receipt& receipt) {
  auto result = receipt.envelope;
  result["document"] = *receipt.document;
  return result;
}

std::optional<Json> SceneStore::requestResult(const std::string& request_id) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const auto found = request_ids_.find(request_id);
  if (found == request_ids_.end()) return {};
  return receiptValue(*found->second);
}

std::size_t SceneStore::obstacleValidations() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return obstacle_validations_;
}

std::size_t SceneStore::yamlObstacleEmissions() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return yaml_.obstacleEmissions();
}

Json SceneStore::command(const Json& request) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  try {
    if (in_hook_) throw SceneError("Scene commands cannot be issued from store hooks");
    fields(request, {"requestId", "expectedEpoch", "expectedRevision", "operation", "obstacle", "id", "document"}, {"operation"});
    if (request.at("operation") == "get") return resultLocked(true);
    const auto rid = request.value("requestId", Json(nullptr));
    static const std::regex identity("[A-Za-z0-9._:-]{1,128}");
    if (!rid.is_string() || !std::regex_match(rid.get_ref<const std::string&>(), identity))
      throw SceneError("Request ID must follow the XRPC identity contract");
    const auto id = rid.get<std::string>();
    const auto fingerprint = dumpJson(sortedFields(request));
    const auto previous = request_ids_.find(id);
    if (previous != request_ids_.end()) {
      if (previous->second->fingerprint != fingerprint) throw SceneError("Request ID was already used for a different operation");
      requests_.splice(requests_.end(), requests_, previous->second);
      return receiptValue(*previous->second);
    }
    const auto expected_epoch = request.value("expectedEpoch", Json(nullptr));
    const auto expected_revision = request.value("expectedRevision", Json(nullptr));
    if (expected_epoch != epoch_ || !expected_revision.is_number_integer() || expected_revision != Json(revision_))
      throw SceneError("Scene changed; refresh before editing");
    Json result;
    try { execute(request); result = resultLocked(true); }
    catch (const SceneError& error) { result = resultLocked(false, error.what()); }
    catch (const std::system_error& error) { result = resultLocked(false, error.what()); }
    catch (const std::invalid_argument& error) { result = resultLocked(false, error.what()); }
    catch (const Json::exception& error) { result = resultLocked(false, error.what()); }
    Json metadata = Json::object();
    for (auto it = result.begin(); it != result.end(); ++it)
      metadata[it.key()] = it.key() == "document" ? Json(nullptr) : it.value();
    requests_.push_back(Receipt{id, fingerprint, std::move(metadata), document_});
    request_ids_[id] = std::prev(requests_.end());
    while (requests_.size() > 256) {
      request_ids_.erase(requests_.front().id);
      requests_.pop_front();
    }
    return result;
  } catch (const SceneError& error) { return resultLocked(false, error.what()); }
  catch (const std::system_error& error) { return resultLocked(false, error.what()); }
  catch (const std::invalid_argument& error) { return resultLocked(false, error.what()); }
  catch (const Json::exception& error) { return resultLocked(false, error.what()); }
}

void SceneStore::execute(const Json& request) {
  const auto& operation = request.at("operation");
  if (frozen_ && (operation.is_array() || operation.is_object()))
    throw SceneError(operation.is_array() ? "unhashable type: 'list'" : "unhashable type: 'dict'");
  if (frozen_ && (operation == "add" || operation == "update" || operation == "delete" || operation == "clear" ||
                 operation == "replace" || operation == "undo" || operation == "redo" || operation == "save" || operation == "reload"))
    throw SceneError(kFrozenError);
  if (operation == "resync") { revision_ = apply(document_); return; }
  if (operation == "save") { checkSource(); save(); return; }
  if (operation == "reload") {
    if (!source_) throw SceneError("This scene has no source YAML configured");
    auto loaded = readDocument(*source_);
    auto replacement = std::make_shared<const Json>(std::move(loaded.first));
    const auto revision = apply(replacement);
    document_ = replacement;
    revision_ = revision;
    saved_document_ = replacement;
    saved_revision_ = revision;
    file_digests_[source_->string()] = std::move(loaded.second);
    undo_stack_.clear();
    redo_stack_.clear();
    return;
  }
  if (operation == "play" || operation == "pause" || operation == "reset") {
    { HookGuard hook(in_hook_); motion_(operation.get<std::string>(), epoch_, revision_); }
    const auto elapsed = sceneTimeLocked();
    playing_ = operation == "play";
    elapsed_ = operation == "reset" ? 0.0 : elapsed;
    started_at_ = nowLocked();
    return;
  }
  if (operation == "undo" || operation == "redo") {
    auto& source = operation == "undo" ? undo_stack_ : redo_stack_;
    auto& target = operation == "undo" ? redo_stack_ : undo_stack_;
    if (source.empty()) throw SceneError("Nothing to " + operation.get<std::string>());
    prepareEdit();
    const auto replacement = source.back();
    const auto revision = apply(replacement);
    target.push_back(document_);
    source.pop_back();
    document_ = replacement;
    revision_ = revision;
    autosave();
    return;
  }
  Json candidate = *document_;
  auto& obstacles = candidate.at("obstacles");
  if (operation == "add" || operation == "update") {
    ++obstacle_validations_;
    auto replacement = obstacle(request.value("obstacle", Json(nullptr)));
    auto found = std::find_if(obstacles.begin(), obstacles.end(), [&](const auto& item) { return item.at("id") == replacement.at("id"); });
    if (operation == "add") {
      if (found != obstacles.end()) throw SceneError("Obstacle ID already exists");
      obstacles.push_back(std::move(replacement));
    } else {
      if (found == obstacles.end()) throw SceneError("Obstacle no longer exists");
      *found = std::move(replacement);
    }
    checkObstacleSet(obstacles);
  } else if (operation == "delete") {
    const auto id = identifier(request.value("id", Json(nullptr)), "Obstacle ID");
    Json remaining = Json::array();
    for (const auto& item : obstacles) if (item.at("id") != id) remaining.push_back(item);
    if (remaining.size() == obstacles.size()) throw SceneError("Obstacle no longer exists");
    candidate["obstacles"] = std::move(remaining);
  } else if (operation == "clear") candidate["obstacles"] = Json::array();
  else if (operation == "replace") {
    candidate = xgc_scene::document(request.value("document", Json(nullptr)));
    obstacle_validations_ += candidate.at("obstacles").size();
  } else throw SceneError("Unsupported scene operation " + detail::pythonRepr(operation));
  if (candidate == *document_) return;
  prepareEdit();
  auto replacement = std::make_shared<const Json>(std::move(candidate));
  const auto revision = apply(replacement);
  undo_stack_.push_back(document_);
  if (undo_stack_.size() > 128) undo_stack_.erase(undo_stack_.begin());
  redo_stack_.clear();
  document_ = replacement;
  revision_ = revision;
  autosave();
}

SceneStore::Revision SceneStore::apply(const Snapshot& candidate) {
  const auto revision = next_revision_++;
  application_known_ = false;
  { HookGuard hook(in_hook_); apply_(*candidate, epoch_, revision); }
  applied_revision_ = revision;
  application_known_ = true;
  return revision;
}

void SceneStore::checkSource() const {
  if (!source_) return;
  const auto current = currentDigest(*source_);
  const auto found = file_digests_.find(source_->string());
  const auto expected = found == file_digests_.end() ? std::optional<std::string>{} : found->second;
  if (current != expected) throw SceneError(kExternalEdit);
}

void SceneStore::prepareEdit() {
  checkSource();
  if (working_file_ && source_ != working_file_) save();
}

void SceneStore::autosave() {
  if (!source_) return;
  try { save(); }
  catch (const SceneError& error) { throw SceneError("Live scene updated, but YAML was not saved: " + std::string(error.what())); }
  catch (const std::system_error& error) { throw SceneError("Live scene updated, but YAML was not saved: " + std::string(error.what())); }
}

void SceneStore::save() {
  if (!save_root_) throw SceneError("This scene has no writable project directory configured");
  const auto target_option = working_file_ ? working_file_ : source_;
  if (!target_option) throw SceneError("This scene has no source YAML configured");
  const auto& target = *target_option;
  const auto relative = target.lexically_relative(*save_root_);
  if (relative.empty() || *relative.begin() == "..") throw SceneError("Save target is outside the configured project directory");
  auto extension = target.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char c) { return std::tolower(c); });
  if (extension != ".yaml" && extension != ".yml") throw SceneError("Save target must be a YAML file");
  if (working_file_) fs::create_directories(target.parent_path());
  if (!fs::is_directory(target.parent_path())) throw SceneError("Save directory does not exist");
  const auto current = currentDigest(target);
  const auto found = file_digests_.find(target.string());
  const auto expected = found == file_digests_.end() ? std::optional<std::string>{} : found->second;
  if (current != expected) throw SceneError(kExternalEdit);
  const auto encoded = yaml_.dump(*document_);
  SaveFile temporary(target);
  if (fs::exists(target)) {
    struct stat info{};
    if (::stat(target.c_str(), &info) != 0) throw SceneError(ioError(errno, target));
    if (::fchmod(temporary.descriptor, info.st_mode & 07777) != 0) throw SceneError(ioError(errno, temporary.path));
  }
  std::size_t written = 0;
  while (written < encoded.size()) {
    const auto count = ::write(temporary.descriptor, encoded.data() + written, encoded.size() - written);
    if (count < 0) {
      if (errno == EINTR) continue;
      throw SceneError(ioError(errno, temporary.path));
    }
    if (count == 0) throw SceneError(ioError(EIO, temporary.path));
    written += static_cast<std::size_t>(count);
  }
  if (::fsync(temporary.descriptor) != 0) throw SceneError(ioError(errno, temporary.path));
  temporary.close();
  if (currentDigest(target) != expected) throw SceneError("Scene file changed while saving");
  try { detail::replaceStoreFile(temporary.path, target); }
  catch (const std::system_error& error) { throw SceneError(ioError(error.code().value(), temporary.path, target)); }
  const int directory = ::open(target.parent_path().c_str(), O_RDONLY);
  if (directory < 0) throw SceneError(ioError(errno, target.parent_path()));
  const int sync = ::fsync(directory);
  const int sync_error = errno;
  const int closed = ::close(directory);
  if (sync != 0) throw SceneError(ioError(sync_error, target.parent_path()));
  if (closed != 0) throw SceneError(ioError(errno, target.parent_path()));
  source_ = target;
  file_digests_[target.string()] = digest(encoded);
  saved_document_ = document_;
  saved_revision_ = revision_;
}

}  // namespace xgc_scene
