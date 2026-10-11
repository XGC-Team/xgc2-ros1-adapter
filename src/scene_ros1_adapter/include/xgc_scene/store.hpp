#pragma once

#include "xgc_scene/common.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace xgc_scene {

inline constexpr char kFrozenError[] =
    "Scene geometry is read-only for this simulator; select an editable scene to change obstacles";

// Reuses obstacle text by value. Like the Python writer, a caller sharing a
// DocumentYaml between threads must serialize calls (SceneStore does so).
class DocumentYaml {
 public:
  std::string dump(const Json& value);
  // Work counters count actual obstacle encodings, including cache misses.
  std::size_t obstacleEmissions() const { return emitted_obstacles_; }

 private:
  struct Item { Json value; std::string text; };
  std::unordered_map<std::string, std::shared_ptr<const Item>> items_;
  std::size_t emitted_obstacles_ = 0;
};

// Python json.dumps(ensure_ascii=False, allow_nan=False) with reusable document
// and obstacle encodings. One encoder may be shared by publishing threads.
class EnvelopeJson {
 public:
  std::string document(const Json& value);
  std::string dumps(const Json& value);
  std::size_t obstacleEmissions() const;

 private:
  std::string documentLocked(const Json& value);
  mutable std::mutex mutex_;
  std::optional<Json> document_;
  std::string document_text_;
  struct Item { Json value; std::string text; };
  std::unordered_map<std::string, std::shared_ptr<const Item>> obstacles_;
  std::size_t emitted_obstacles_ = 0;
};

// Uses an encoder private to the calling thread, so repeated publishes share
// unchanged text without sharing mutable caches between threads.
std::string envelopeJson(const Json& value);

class SceneStore {
 public:
  using Revision = std::uint64_t;
  using Apply = std::function<void(const Json&, const std::string&, Revision)>;
  using Motion = std::function<void(const std::string&, const std::string&, Revision)>;
  using Clock = std::function<double()>;

  explicit SceneStore(const Json& initial, std::optional<std::filesystem::path> source = {},
                      std::optional<std::filesystem::path> save_root = {}, Apply apply = {},
                      Clock clock = {}, std::optional<std::string> source_digest = {},
                      bool frozen = false, std::optional<std::filesystem::path> working_file = {},
                      Motion motion = {});

  Json status() const;
  Json envelope() const;
  Json states() const;
  double sceneTime() const;
  Json command(const Json& request);
  // A retained receipt is a snapshot of that command, including failures.
  // Receipt reads do not change command replay recency.
  std::optional<Json> requestResult(const std::string& request_id) const;
  // Counts the incremental edit work for the scaling contract.
  std::size_t obstacleValidations() const;
  std::size_t yamlObstacleEmissions() const;

  // One recursive mutex serializes all reads, commands, IO and hooks. Hooks
  // run synchronously under it and may read this store from the same thread.
  // A hook must not wait for another thread to access the store. Nested
  // commands from apply or motion hooks return an explicit failure. The clock
  // callable must only return the time and must not call back into the store.
  // Documents passed to hooks are immutable; returned JSON values are copies.

 private:
  using Snapshot = std::shared_ptr<const Json>;
  struct Receipt {
    std::string id;
    std::string fingerprint;
    Json envelope;
    Snapshot document;
  };

  Json statusLocked() const;
  Json envelopeLocked() const;
  Json resultLocked(bool success, const std::string& error = {}) const;
  double sceneTimeLocked() const;
  double nowLocked() const;
  void execute(const Json& request);
  Revision apply(const Snapshot& candidate);
  void prepareEdit();
  void checkSource() const;
  void autosave();
  void save();
  static Json receiptValue(const Receipt& receipt);

  mutable std::recursive_mutex mutex_;
  mutable bool in_hook_ = false;
  bool frozen_;
  std::optional<std::filesystem::path> working_file_;
  Snapshot document_;
  std::string epoch_;
  Revision revision_ = 1;
  Revision applied_revision_ = 0;
  bool application_known_ = false;
  Revision next_revision_ = 2;
  Revision saved_revision_ = 1;
  Snapshot saved_document_;
  std::optional<std::filesystem::path> source_;
  std::optional<std::filesystem::path> save_root_;
  std::unordered_map<std::string, std::string> file_digests_;
  Apply apply_;
  Motion motion_;
  Clock clock_;
  bool playing_ = false;
  double elapsed_ = 0;
  double started_at_ = 0;
  std::vector<Snapshot> undo_stack_;
  std::vector<Snapshot> redo_stack_;
  std::list<Receipt> requests_;
  std::unordered_map<std::string, std::list<Receipt>::iterator> request_ids_;
  DocumentYaml yaml_;
  std::size_t obstacle_validations_ = 0;
};

}  // namespace xgc_scene
