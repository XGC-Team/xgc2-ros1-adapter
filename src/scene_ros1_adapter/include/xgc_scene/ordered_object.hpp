#pragma once

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace xgc_scene {

// nlohmann-json 3.7 has no ordered_json. Its object container is the only
// customization needed to keep Python dict insertion order in the JSON model.
template <class Key, class Value, class Compare = std::less<Key>,
          class Allocator = std::allocator<std::pair<const Key, Value>>>
class OrderedObject
    : public std::vector<std::pair<Key, Value>,
                         typename std::allocator_traits<Allocator>::template rebind_alloc<std::pair<Key, Value>>> {
  using Base = std::vector<std::pair<Key, Value>,
                           typename std::allocator_traits<Allocator>::template rebind_alloc<std::pair<Key, Value>>>;

 public:
  using key_type = Key;
  using mapped_type = Value;
  using key_compare = Compare;
  using typename Base::value_type;
  using typename Base::iterator;
  using typename Base::const_iterator;
  using Base::Base;
  using Base::erase;

  iterator find(const Key& key) {
    return std::find_if(this->begin(), this->end(), [&](const auto& item) { return item.first == key; });
  }
  const_iterator find(const Key& key) const {
    return std::find_if(this->begin(), this->end(), [&](const auto& item) { return item.first == key; });
  }
  std::size_t count(const Key& key) const { return find(key) != this->end(); }
  Value& at(const Key& key) {
    auto found = find(key);
    if (found == this->end()) throw std::out_of_range("object key not found");
    return found->second;
  }
  const Value& at(const Key& key) const {
    auto found = find(key);
    if (found == this->end()) throw std::out_of_range("object key not found");
    return found->second;
  }
  Value& operator[](const Key& key) { return emplace(key, Value{}).first->second; }
  Value& operator[](Key&& key) { return emplace(std::move(key), Value{}).first->second; }

  template <class... Args>
  std::pair<iterator, bool> emplace(Args&&... args) {
    value_type item(std::forward<Args>(args)...);
    auto found = find(item.first);
    if (found != this->end()) return {found, false};
    this->push_back(std::move(item));
    return {std::prev(this->end()), true};
  }
  std::pair<iterator, bool> insert(const value_type& item) { return emplace(item); }
  std::pair<iterator, bool> insert(value_type&& item) { return emplace(std::move(item)); }
  template <class InputIterator>
  void insert(InputIterator first, InputIterator last) {
    for (; first != last; ++first) emplace(*first);
  }
  std::size_t erase(const Key& key) {
    auto found = find(key);
    if (found == this->end()) return 0;
    Base::erase(found);
    return 1;
  }

  // Dict equality ignores insertion order; serialization preserves it.
  friend bool operator==(const OrderedObject& a, const OrderedObject& b) {
    if (a.size() != b.size()) return false;
    for (const auto& item : a) {
      auto found = b.find(item.first);
      if (found == b.end() || item.second != found->second) return false;
    }
    return true;
  }
  friend bool operator!=(const OrderedObject& a, const OrderedObject& b) { return !(a == b); }
};

}  // namespace xgc_scene
