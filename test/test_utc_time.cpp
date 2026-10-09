#include <gtest/gtest.h>
#include <cstdlib>
#include <ctime>
#include <string>
#include "pandarGeneral/utc_time.h"

TEST(UtcTime, SensorCalendarIsIndependentOfTimezone) {
  const char* original = std::getenv("TZ");
  const bool had_tz = original != nullptr;
  const std::string saved = original ? original : "";
  for (const char* zone : {"Etc/UTC", "Asia/Tokyo", "America/New_York"}) {
    setenv("TZ", zone, 1);
    tzset();
    for (const auto& sample : {std::pair<int, std::time_t>{0, 1767225600},
                               std::pair<int, std::time_t>{6, 1782864000}}) {
      std::tm utc{};
      utc.tm_year = 126;
      utc.tm_mon = sample.first;
      utc.tm_mday = 1;
      EXPECT_EQ(hesai::UtcToUnixSeconds(utc), sample.second) << zone;
      EXPECT_STREQ(std::getenv("TZ"), zone);
      EXPECT_EQ(utc.tm_mon, sample.first);
    }
  }
  unsetenv("TZ");
  std::tm epoch{};
  epoch.tm_year = 70;
  epoch.tm_mday = 1;
  EXPECT_EQ(hesai::UtcToUnixSeconds(epoch), 0);
  EXPECT_EQ(std::getenv("TZ"), nullptr);
  if (had_tz) setenv("TZ", saved.c_str(), 1);
  tzset();
}
