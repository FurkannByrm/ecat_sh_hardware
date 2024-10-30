#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include <sys/time.h>
#include <chrono>

template <class TickType, class Period = std::ratio<1>>
constexpr auto /*std::chrono::duration<TickType, Period>*/
timespecToChronoDuration(const timespec &ts) {
  return std::chrono::duration_cast<std::chrono::duration<TickType, Period>(
      std::chrono::seconds(ts.tv_sec) + std::chrono::nanoseconds(ts.tv_nsec))>;
}

template <class TickType, class Period = std::ratio<1>>
constexpr std::timespec
chronoDurationToTimespec(std::chrono::duration<TickType, Period> duration) {
  std::chrono::seconds durAsSec =
      std::chrono::duration_cast<std::chrono::seconds>(duration);
  std::chrono::nanoseconds durAsNsec =
      std::chrono::duration_cast<std::chrono::nanoseconds>(duration - durAsSec);

  return {.tv_sec = durAsSec.count(), .tv_nsec = durAsNsec.count()};
}

template <class ClockType, class TickType, class Period = std::ratio<1>>
constexpr auto timespecToChronoTimePoint(const std::timespec &ts) {
  return std::chrono::time_point<ClockType,
                                 std::chrono::duration<TickType, Period>>{
      timespecToChronoDuration<TickType, Period>(ts)};
}

template <class ClockType, class TickType, class Period = std::ratio<1>>
constexpr std::timespec timepointToTimespec(
    std::chrono::time_point<ClockType, std::chrono::duration<TickType, Period>>
        tp) {
  auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(tp);
  auto nsecs = std::chrono::time_point_cast<std::chrono::nanoseconds>(tp) -
               std::chrono::time_point_cast<std::chrono::nanoseconds>(seconds);

  return {.tv_sec = seconds.time_since_epoch().count(),
          .tv_nsec = nsecs.count()};
}



namespace ecat_sh_hardware {

#define DistributedClockHelper DistributedClockHelper_t

constexpr auto NANOSEC_PER_SEC = 1000000000L;


struct DistributedClockHelper_t
{
    struct timespec cycleTime;
    struct timespec wakeupTime;
    struct timespec currentTime;

    long periodNanoSec;
    
    clockid_t clock;

    int referenceClockCounter;

};

long int period_nanosec(int frequency);

uint64_t timespecToNanoSec(struct timespec ts);

struct timespec addTimespec(struct timespec time1, struct timespec time2);

int sleep_task(clockid_t clock_to_use, int flag, const struct timespec* required_duration, struct timespec* remaining_duration);

enum class Error {
  NoError,
  shmOpenError,
  ftruncateError,
  mmapError,
  semOpenError,
  sendEcObjectError
};
} // namespace ecat_sh_hardware

struct PeriodicSleeper
{
  
};

#endif // UTILS_HPP_
