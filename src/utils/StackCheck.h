#pragma once

#include <cstdint>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

#include "platform/PlatformUtils.h"

class StackCheck
{
public:
  static StackCheck& inst()
  {
    static StackCheck instance;
    return instance;
  }

  bool check() const
  {
    const auto address = stackAddress();
    const auto used = address < origin ? origin - address : address - origin;
    return used >= limit;
  }

private:
  StackCheck() : limit(PlatformUtils::stackLimit()), origin(stackAddress()) {}

  static std::uintptr_t stackAddress()
  {
#if defined(_MSC_VER)
    return reinterpret_cast<std::uintptr_t>(_AddressOfReturnAddress());
#elif defined(__GNUC__) && !defined(__EMSCRIPTEN__)
    return reinterpret_cast<std::uintptr_t>(__builtin_frame_address(0));
#else
    unsigned char marker;
    return reinterpret_cast<std::uintptr_t>(&marker);
#endif
  }

  const unsigned long limit;
  const std::uintptr_t origin;
};
