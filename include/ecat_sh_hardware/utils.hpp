#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include <sys/time.h>
#include <chrono>

template <typename E>
constexpr auto to_integral(E e) -> typename std::underlying_type<E>::type {
  return static_cast<typename std::underlying_type<E>::type>(e);
}

enum class CIA402_State
{
  UNKNOWN_STATE = -1,
  START = 0,
  NOT_READY_TO_SWITCH_ON = 1,
  SWITCH_ON_DISABLED = 2,
  READY_TO_SWITCH_ON = 3,
  SWITCH_ON = 4,
  OPERATION_ENABLED = 5,
  FAULT_REACTION_ACTIVE = 6,
  FAULT = 7,
  QUICK_STOP_ACTIVE = 8
};

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
