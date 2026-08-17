#pragma once

#include "models/drone_link.h"
#include <array>
#include <cstdint>
#include <cstring>

namespace comms
{

  // Стурктура, що описує один кадр протоколу drone_link.
  // Кадр складається з типу пакета, довжини і масиву payload, який містить дані пакета
  // Можна конвертувати payload у будь-яку структуру, що відповідає типу пакета
  struct Frame
  {
    bool ok{false};
    dlink::PacketType type{};
    uint8_t len{0};
    std::array<uint8_t, 260> payload{};

    template <class T>
    T as() const
    {
      T value{};
      std::memcpy(&value, payload.data(), sizeof(T));
      return value;
    }
  };

}
