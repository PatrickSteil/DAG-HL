#pragma once
#include <cstdint>
#include <limits>
#include <vector>

template <typename T>
class TimestampedVector {
 public:
  TimestampedVector() = default;

  explicit TimestampedVector(std::size_t n, const T& default_value = T())
      : values(n), stamps(n, 0), defaultValue(default_value), currentStamp(1) {}

  void resize(std::size_t n) {
    values.resize(n);
    stamps.resize(n, 0);
  }

  void reset() {
    if (++currentStamp == 0) {
      std::fill(stamps.begin(), stamps.end(), 0);
      currentStamp = 1;
    }
  }

  inline bool isActive(std::size_t i) const {
    return stamps[i] == currentStamp;
  }

  inline const T& get(std::size_t i) const {
    return (stamps[i] == currentStamp) ? values[i] : defaultValue;
  }

  inline void set(std::size_t i, const T& value) {
    values[i] = value;
    stamps[i] = currentStamp;
  }

  inline T& operator[](std::size_t i) {
    if (stamps[i] != currentStamp) {
      stamps[i] = currentStamp;
      values[i] = defaultValue;
    }
    return values[i];
  }

 private:
  std::vector<T> values;
  std::vector<uint16_t> stamps;
  T defaultValue;
  uint16_t currentStamp;
};
