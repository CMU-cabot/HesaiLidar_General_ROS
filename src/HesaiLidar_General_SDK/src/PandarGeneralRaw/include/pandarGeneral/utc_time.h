#pragma once

#include <ctime>

namespace hesai {
// Sensor calendar fields are UTC, independent of the process timezone. Take a
// copy because timegm normalizes its argument. Keep the SDK's explicit offset
// at the call site; never change the process-wide TZ environment.
inline std::time_t UtcToUnixSeconds(std::tm utc) {
  return ::timegm(&utc);
}
}  // namespace hesai
