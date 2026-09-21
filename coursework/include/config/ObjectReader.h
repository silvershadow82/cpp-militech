#pragma once

#include "Types.h"
#include "config/FileConfigLoader.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <initializer_list>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace follow::config::detail {

inline void require(bool ok, const std::string &message)
{
  if (!ok) {
    throw ConfigError(message);
  }
}

// One JSON object of a config file: rejects unknown keys and names the full key path in every error.
class ObjectReader {
public:
  ObjectReader(const nlohmann::json &object, std::string path, std::initializer_list<std::string_view> allowed)
    : object(object)
    , path(std::move(path))
  {
    require(this->object.is_object(), this->label() + ": expected an object");
    for (const auto &item : this->object.items()) {
      require(std::find(allowed.begin(), allowed.end(), item.key()) != allowed.end(), this->label(item.key()) + ": unknown key");
    }
  }

  // Nested object; an absent key reads as an empty object, so every field keeps its default.
  ObjectReader child(std::string_view key, std::initializer_list<std::string_view> allowed) const
  {
    static const nlohmann::json empty = nlohmann::json::object();
    const nlohmann::json *value = this->raw(key);
    return ObjectReader(value ? *value : empty, this->label(key), allowed);
  }

  // The value under key, or nullptr if absent.
  const nlohmann::json *raw(std::string_view key) const
  {
    auto it = this->object.find(std::string(key));
    return it == this->object.end() ? nullptr : &*it;
  }

  template <class T>
  void read(std::string_view key, T &out) const
  {
    if (const nlohmann::json *value = this->raw(key)) {
      try {
        out = value->get<T>();
      }
      catch (const nlohmann::json::exception &e) {
        throw ConfigError(this->label(key) + ": " + e.what());
      }
    }
  }

  template <class T>
  void readRequired(std::string_view key, T &out) const
  {
    require(this->raw(key) != nullptr, this->label(key) + ": required");
    this->read(key, out);
  }

  void readMs(std::string_view key, std::chrono::milliseconds &out) const
  {
    int64_t ms = out.count();
    this->read(key, ms);
    require(ms >= 0, this->label(key) + ": must not be negative");
    out = std::chrono::milliseconds{ms};
  }

  // Absent keeps the value, null clears it.
  void readOptional(std::string_view key, std::optional<double> &out) const
  {
    const nlohmann::json *value = this->raw(key);
    if (!value) {
      return;
    }
    if (value->is_null()) {
      out.reset();
      return;
    }
    double number = 0.0;
    this->read(key, number);
    out = number;
  }

  // [width, height], both positive.
  void readSize(std::string_view key, int &width, int &height) const
  {
    std::vector<int> size{width, height};
    this->read(key, size);
    require(size.size() == 2 && size[0] > 0 && size[1] > 0, this->label(key) + ": expected [width, height]");
    width = size[0];
    height = size[1];
  }

  // Required [x, y] into a ground point (z = 0).
  void readPoint(std::string_view key, models::Vec3 &out) const
  {
    std::vector<double> point;
    this->readRequired(key, point);
    require(point.size() == 2, this->label(key) + ": expected [x, y]");
    out = {point[0], point[1], 0.0};
  }

  std::string label(std::string_view key = {}) const
  {
    if (key.empty()) {
      return this->path.empty() ? std::string("<root>") : this->path;
    }
    return this->path.empty() ? std::string(key) : this->path + "." + std::string(key);
  }

private:
  const nlohmann::json &object;
  std::string path;
};

}  // namespace follow::config::detail
