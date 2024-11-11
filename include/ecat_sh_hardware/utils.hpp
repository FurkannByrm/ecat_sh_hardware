#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include <sys/time.h>
#include <chrono>

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
